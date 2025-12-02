import tkinter as tk
from tkinter import filedialog, messagebox
from PIL import Image, ImageTk, ImageDraw
import yaml
import os

class MapEditorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("ROS 2 Map Cleaner (Zoom & Tools)")
        self.root.geometry("1100x850")

        # --- 変数初期化 ---
        self.image_path = None
        self.original_image = None # 元の画像データ (Scale 1.0)
        self.tk_image = None       # 表示用の画像データ (Scale x.x)
        self.draw = None
        
        self.brush_size_map = 5    # マップ上のピクセル単位でのブラシサイズ
        self.current_scale = 1.0   # 現在の表示倍率
        
        # 描画モード (255=Free/White, 0=Occupied/Black, 205=Unknown/Gray)
        self.draw_color_var = tk.IntVar(value=255)
        
        # ツール選択
        self.tool_var = tk.StringVar(value="free")
        
        # Box塗りつぶしオプション
        self.fill_var = tk.BooleanVar(value=False)

        # 描画操作用の一時変数
        self.start_pos = None  # (x, y) Line/Boxの始点 (Canvas座標系ではなくMap座標系で保持)
        self.cursor_id = None
        self.preview_id = None 

        # UIの構築
        self._setup_ui()

    def _setup_ui(self):
        # --- ツールバーエリア ---
        toolbar = tk.Frame(self.root, bd=1, relief=tk.RAISED)
        toolbar.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)

        # 1. ファイル操作
        frame_file = tk.LabelFrame(toolbar, text="File", padx=5, pady=2)
        frame_file.pack(side=tk.LEFT, padx=5, fill=tk.Y)
        tk.Button(frame_file, text="Load", command=self.load_file).pack(side=tk.LEFT, padx=2)
        tk.Button(frame_file, text="Save", command=self.save_file).pack(side=tk.LEFT, padx=2)

        # 2. 描画色
        frame_color = tk.LabelFrame(toolbar, text="Color", padx=5, pady=2)
        frame_color.pack(side=tk.LEFT, padx=5, fill=tk.Y)
        modes = [("Free", 255), ("Wall", 0), ("Unknown", 205)]
        for text, val in modes:
            tk.Radiobutton(frame_color, text=text, variable=self.draw_color_var, value=val).pack(anchor=tk.W)

        # 3. 描画ツール
        frame_tool = tk.LabelFrame(toolbar, text="Tool", padx=5, pady=2)
        frame_tool.pack(side=tk.LEFT, padx=5, fill=tk.Y)
        tools = [("Pen", "free"), ("Line", "line"), ("Box", "box")]
        for text, mode in tools:
            tk.Radiobutton(frame_tool, text=text, variable=self.tool_var, value=mode, command=self.reset_state).pack(anchor=tk.W)
        tk.Checkbutton(frame_tool, text="Fill Box", variable=self.fill_var).pack(anchor=tk.W)

        # 4. ブラシサイズ
        frame_brush = tk.LabelFrame(toolbar, text="Brush Size", padx=5, pady=2)
        frame_brush.pack(side=tk.LEFT, padx=5, fill=tk.Y)
        self.scale_brush = tk.Scale(frame_brush, from_=1, to=50, orient=tk.HORIZONTAL, command=self.update_brush_size)
        self.scale_brush.set(self.brush_size_map)
        self.scale_brush.pack(side=tk.LEFT)

        # 5. ズームコントロール
        frame_zoom = tk.LabelFrame(toolbar, text="Zoom", padx=5, pady=2)
        frame_zoom.pack(side=tk.LEFT, padx=5, fill=tk.Y)
        self.scale_zoom = tk.Scale(frame_zoom, from_=0.1, to=5.0, resolution=0.1, orient=tk.HORIZONTAL, command=self.on_slider_zoom)
        self.scale_zoom.set(1.0)
        self.scale_zoom.pack(side=tk.LEFT)

        # --- キャンバスエリア ---
        self.canvas_frame = tk.Frame(self.root)
        self.canvas_frame.pack(fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(self.canvas_frame, bg="#808080", cursor="none")
        self.scroll_x = tk.Scrollbar(self.canvas_frame, orient="horizontal", command=self.canvas.xview)
        self.scroll_y = tk.Scrollbar(self.canvas_frame, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(xscrollcommand=self.scroll_x.set, yscrollcommand=self.scroll_y.set)

        self.scroll_x.pack(side=tk.BOTTOM, fill=tk.X)
        self.scroll_y.pack(side=tk.RIGHT, fill=tk.Y)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # --- イベントバインド ---
        self.canvas.bind("<Button-1>", self.on_left_click)
        self.canvas.bind("<B1-Motion>", self.on_left_drag)
        self.canvas.bind("<Motion>", self.on_mouse_move)
        
        self.canvas.bind("<Button-3>", self.on_right_click)
        self.canvas.bind("<B3-Motion>", self.on_right_drag)

        # ズーム用イベント (Windows/Linux対応)
        # Shift + MouseWheel
        self.canvas.bind("<Shift-MouseWheel>", self.on_mouse_wheel) 
        # Linux (X11) でのホイール対応 (Shift + Button-4/5)
        self.canvas.bind("<Shift-Button-4>", lambda e: self.on_mouse_wheel(e, 1))
        self.canvas.bind("<Shift-Button-5>", lambda e: self.on_mouse_wheel(e, -1))

        self.root.bind("<Escape>", self.cancel_operation)

    # -------------------------------------------------------------------------
    # 状態リセット
    # -------------------------------------------------------------------------
    def reset_state(self):
        self.start_pos = None
        self.canvas.delete("preview")

    def cancel_operation(self, event=None):
        if self.start_pos:
            self.reset_state()
            messagebox.showinfo("Info", "Operation cancelled.")

    def update_brush_size(self, val):
        self.brush_size_map = int(val)
        # ブラシサイズを変えたらカーソルも更新したいので、現在のマウス位置がわかれば再描画したいが
        # 簡易的に次のマウス移動イベントで更新されるのを待つ

    # -------------------------------------------------------------------------
    # ズーム処理
    # -------------------------------------------------------------------------
    def on_slider_zoom(self, val):
        """スライダー操作時のズーム（中心基準）"""
        new_scale = float(val)
        if new_scale != self.current_scale:
            self.apply_zoom(new_scale, center=None)

    def on_mouse_wheel(self, event, direction=None):
        """Shift+ホイール操作時のズーム（マウス位置基準）"""
        if not self.original_image:
            return

        # 拡大縮小の方向決定
        if direction is None:
            # Windows / MacOS
            delta = event.delta
        else:
            # Linux Button-4/5
            delta = direction * 120

        scale_factor = 1.1 if delta > 0 else 0.9
        new_scale = self.current_scale * scale_factor
        
        # 範囲制限
        new_scale = max(0.1, min(new_scale, 5.0))
        
        # スライダーの値を更新 (これが on_slider_zoom をトリガーしないように注意が必要だが
        # TkinterのScaleはsetではcommandを呼ばないので安全)
        self.scale_zoom.set(new_scale)
        
        # マウス位置を中心（ピボット）にしてズーム適用
        self.apply_zoom(new_scale, pivot_event=event)

    def apply_zoom(self, new_scale, center=None, pivot_event=None):
        """
        ズームの実行処理
        pivot_event: マウスイベント（ここにマウスカーソルがある）
        """
        if not self.original_image:
            return

        old_scale = self.current_scale
        self.current_scale = new_scale

        # 1. 画像の表示倍率を変更して再描画
        self.update_image_display()

        # 2. スクロール位置の調整
        if pivot_event:
            # マウス位置に向けてズームする計算
            
            # マウスのキャンバス上の絶対座標（スクロール考慮済み）
            mouse_canvas_x = self.canvas.canvasx(pivot_event.x)
            mouse_canvas_y = self.canvas.canvasy(pivot_event.y)

            # 画像左上からの比率 (0.0 ~ 1.0) を計算
            width_old = self.original_image.width * old_scale
            height_old = self.original_image.height * old_scale
            
            ratio_x = mouse_canvas_x / width_old
            ratio_y = mouse_canvas_y / height_old

            # 新しいサイズ
            width_new = self.original_image.width * new_scale
            height_new = self.original_image.height * new_scale

            # 新しいキャンバス上の目標座標
            new_canvas_x = ratio_x * width_new
            new_canvas_y = ratio_y * height_new

            # スクロールバーの位置合わせ
            # (new_canvas_x - pivot_event.x) が新しい左上の座標になるべき
            # xview_moveto は "全幅に対する左端の割合" を指定する
            
            # 画面に見えている幅・高さ
            view_w = self.canvas.winfo_width()
            view_h = self.canvas.winfo_height()
            
            # 目標の左上座標
            target_left = new_canvas_x - pivot_event.x
            target_top = new_canvas_y - pivot_event.y
            
            # 割合に変換してセット
            self.canvas.xview_moveto(target_left / width_new)
            self.canvas.yview_moveto(target_top / height_new)

    def update_image_display(self):
        """現在の倍率で画像をリサイズして表示"""
        if not self.original_image:
            return

        w = int(self.original_image.width * self.current_scale)
        h = int(self.original_image.height * self.current_scale)
        
        # 処理速度とドット絵らしさを優先して NEAREST 近傍法を使用
        resized_image = self.original_image.resize((w, h), Image.NEAREST)
        self.tk_image = ImageTk.PhotoImage(resized_image)

        # 既存の画像IDがあれば更新、なければ作成
        # ただしCanvas上の描画物(drawn_pixels)との兼ね合いがあるため、
        # ズーム時は「一時的な描画(Canvas上の円など)」はすべて消して、画像自体を再生成する方式とする
        self.canvas.delete("all") 
        
        self.canvas.config(scrollregion=(0, 0, w, h))
        self.canvas.create_image(0, 0, image=self.tk_image, anchor="nw", tag="map_image")
        
        # プレビューなどの再描画は次のマウスムーブで行われるのでここでは画像のみ

    # -------------------------------------------------------------------------
    # ファイル操作
    # -------------------------------------------------------------------------
    def load_file(self):
        file_path = filedialog.askopenfilename(
            filetypes=[("Map Files", "*.yaml *.pgm *.png"), ("All Files", "*.*")]
        )
        if not file_path:
            return

        ext = os.path.splitext(file_path)[1].lower()
        try:
            target_image_path = file_path
            if ext == '.yaml':
                with open(file_path, 'r') as f:
                    data = yaml.safe_load(f)
                    image_filename = data.get('image', '')
                    target_image_path = os.path.join(os.path.dirname(file_path), image_filename)
            
            self.image_path = target_image_path
            self.original_image = Image.open(self.image_path).convert("L")
            self.draw = ImageDraw.Draw(self.original_image)
            
            # リセット
            self.current_scale = 1.0
            self.scale_zoom.set(1.0)
            self.update_image_display()
            self.reset_state()
            
            messagebox.showinfo("Success", f"Loaded: {os.path.basename(self.image_path)}")

        except Exception as e:
            messagebox.showerror("Error", f"Failed to load file:\n{e}")

    def save_file(self):
        if not self.original_image: return
        save_path = filedialog.asksaveasfilename(
            initialfile=os.path.basename(self.image_path),
            defaultextension=".pgm",
            filetypes=[("PGM Map", "*.pgm"), ("PNG Map", "*.png")]
        )
        if save_path:
            try:
                self.original_image.save(save_path)
                messagebox.showinfo("Saved", f"Map saved to:\n{save_path}")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to save:\n{e}")

    # -------------------------------------------------------------------------
    # 座標変換ヘルパー
    # -------------------------------------------------------------------------
    def canvas_to_map(self, cx, cy):
        """Canvas座標(Zoomed) -> Map座標(Original)"""
        return int(cx / self.current_scale), int(cy / self.current_scale)

    def map_to_canvas(self, mx, my):
        """Map座標(Original) -> Canvas座標(Zoomed)"""
        return mx * self.current_scale, my * self.current_scale

    # -------------------------------------------------------------------------
    # マウスイベント
    # -------------------------------------------------------------------------
    def on_mouse_move(self, event):
        if not self.original_image: return

        cx = self.canvas.canvasx(event.x)
        cy = self.canvas.canvasy(event.y)
        
        # 1. ブラシカーソルの描画
        # 表示上のサイズ = マップ上のサイズ * 倍率
        r_screen = self.brush_size_map * self.current_scale
        
        if self.cursor_id:
            self.canvas.delete(self.cursor_id)
        
        outline_col = "blue" if self.start_pos else "red"
        
        self.cursor_id = self.canvas.create_oval(
            cx - r_screen, cy - r_screen, cx + r_screen, cy + r_screen,
            outline=outline_col, width=2, tag="cursor"
        )

        # 2. Line/Boxツールのプレビュー
        if self.start_pos:
            self.canvas.delete("preview")
            
            # 始点（マップ座標）を現在のキャンバス座標に変換
            sx_map, sy_map = self.start_pos
            sx_screen, sy_screen = self.map_to_canvas(sx_map, sy_map)
            
            tool = self.tool_var.get()
            line_width = max(1, int(self.brush_size_map * self.current_scale * 2))

            if tool == "line":
                self.canvas.create_line(sx_screen, sy_screen, cx, cy, fill="blue", width=line_width, stipple="gray50", tag="preview")
            elif tool == "box":
                if self.fill_var.get():
                    self.canvas.create_rectangle(sx_screen, sy_screen, cx, cy, fill="blue", outline="blue", stipple="gray25", tag="preview")
                else:
                    self.canvas.create_rectangle(sx_screen, sy_screen, cx, cy, outline="blue", width=2, dash=(4, 4), tag="preview")

    def on_left_click(self, event):
        if not self.original_image: return

        # Canvas座標を取得
        cx = self.canvas.canvasx(event.x)
        cy = self.canvas.canvasy(event.y)
        
        # Map座標に変換
        mx, my = self.canvas_to_map(cx, cy)
        
        tool = self.tool_var.get()
        color = self.draw_color_var.get()

        if tool == "free":
            self._draw_point_on_map(mx, my, color)
            # クリック単発でもCanvas上の見た目を更新(点描画)
            self._draw_temp_visual(mx, my, color)
        
        elif tool in ["line", "box"]:
            if self.start_pos is None:
                self.start_pos = (mx, my) # マップ座標で保持
            else:
                self._execute_tool_draw(self.start_pos, (mx, my), tool, color)
                self.start_pos = None
                self.canvas.delete("preview")

    def on_left_drag(self, event):
        # Free描画のみ対応
        if self.tool_var.get() == "free" and self.original_image:
            cx = self.canvas.canvasx(event.x)
            cy = self.canvas.canvasy(event.y)
            mx, my = self.canvas_to_map(cx, cy)
            
            self._draw_point_on_map(mx, my, self.draw_color_var.get())
            self._draw_temp_visual(mx, my, self.draw_color_var.get())
            
            self.on_mouse_move(event)

    def on_right_click(self, event):
        # ショートカット (Wall)
        if self.tool_var.get() == "free" and self.original_image:
            cx = self.canvas.canvasx(event.x)
            cy = self.canvas.canvasy(event.y)
            mx, my = self.canvas_to_map(cx, cy)
            self._draw_point_on_map(mx, my, 0)
            self._draw_temp_visual(mx, my, 0)

    def on_right_drag(self, event):
        if self.tool_var.get() == "free" and self.original_image:
            cx = self.canvas.canvasx(event.x)
            cy = self.canvas.canvasy(event.y)
            mx, my = self.canvas_to_map(cx, cy)
            self._draw_point_on_map(mx, my, 0)
            self._draw_temp_visual(mx, my, 0)
            self.on_mouse_move(event)

    # -------------------------------------------------------------------------
    # 描画処理 (Original Imageへの書き込み)
    # -------------------------------------------------------------------------
    def _draw_point_on_map(self, mx, my, color):
        r = self.brush_size_map
        self.draw.ellipse([mx-r, my-r, mx+r, my+r], fill=color, outline=color)

    def _draw_temp_visual(self, mx, my, color):
        """
        Canvas上に一時的な描画を行う（重くなるのでPillow再変換はsave/zoom時のみ）
        ただしズーム中は座標がずれるので、Visualは「今のScale」に合わせて描く
        """
        cx, cy = self.map_to_canvas(mx, my)
        r_screen = self.brush_size_map * self.current_scale
        
        fill_col = f"#{color:02x}{color:02x}{color:02x}"
        self.canvas.create_oval(cx-r_screen, cy-r_screen, cx+r_screen, cy+r_screen, 
                                fill=fill_col, outline=fill_col, tag="drawn_pixels")

    def _execute_tool_draw(self, start_map, end_map, tool, color):
        sx, sy = start_map
        ex, ey = end_map
        width = self.brush_size_map * 2
        
        # 1. Pillowに描画 (確定)
        if tool == "line":
            self.draw.line([sx, sy, ex, ey], fill=color, width=width)
        elif tool == "box":
            if self.fill_var.get():
                self.draw.rectangle([sx, sy, ex, ey], fill=color, outline=None)
            else:
                self.draw.rectangle([sx, sy, ex, ey], fill=None, outline=color, width=self.brush_size_map)
        
        # 2. Canvasの画像を更新
        # 直線やBoxは一時描画で再現するのが面倒(特にZoom対応)なので、
        # 確定したタイミングで画像を再生成して表示をリフレッシュする
        self.update_image_display()

if __name__ == "__main__":
    root = tk.Tk()
    app = MapEditorApp(root)
    root.mainloop()
