import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import re

# 1. 探したい単語の辞書を定義（デフォルト用、main.pyから渡される想定）
target_dict = {
    "name": ["tom", "alice", "jon", "田中", "佐藤"],
    "drink": ["cola", "coffe", "milk", "コーラ", "コーヒー"]
}

# 2. フォーマットがバラバラな文章のリスト（例）
sample_texts = [
    "LIKE coffee ,TOm",
    "Alice loves MILK!",  # 大文字混じり
    "My name is jon I like coca cola",
    "jonは何も飲みませんでした。"
]

def search_keywords(text, dictionary):
    found_result = {"name": [], "drink": []}
    text_lower = text.lower()
    
    for category, items in dictionary.items():
        if isinstance(items, dict):
            # itemsが辞書の場合（代表名: [シノニムのリスト]）
            for primary_word, synonyms in items.items():
                for syn in synonyms:
                    clean_word = syn.strip().lower()
                    # 英語の単語境界(\b)を使って検索。日本語の場合は境界が効かない場合があるので部分一致もフォールバック
                    pattern = r'\b' + re.escape(clean_word) + r'\b'
                    if re.search(pattern, text_lower) or clean_word in text_lower:
                        if primary_word not in found_result[category]:
                            found_result[category].append(primary_word)
                        break
        else:
            # itemsがリストの場合（従来の単純なリスト）
            for word in items:
                clean_word = word.strip().lower()
                pattern = r'\b' + re.escape(clean_word) + r'\b'
                if re.search(pattern, text_lower) or clean_word in text_lower:
                    capitalized = word.strip().capitalize()
                    if capitalized not in found_result[category]:
                        found_result[category].append(capitalized)
                
    return found_result

if __name__ == '__main__':
    # --- 実行と結果の表示 ---
    print("【抽出結果】")
    for i, text in enumerate(sample_texts, 1):
        result = search_keywords(text, target_dict)
        print(f"\n文章 {i}: 「{text}」")
        print(f"  見つかった name  : {result['name']}")
        print(f"  見つかった drink : {result['drink']}")
