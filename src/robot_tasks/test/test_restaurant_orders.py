from types import SimpleNamespace

from robot_tasks.restaurant import (
    cb_state_check_order,
    cb_state_check_order_confirmation,
    cb_state_order_report,
)


OBJECTS = {
    "tea_bag": [["tea", "tea bag", "t bag"], 0.1],
    "potato_chips": [["chips", "potato chips"], 0.2],
}


def test_check_order_uses_object_names_once_for_matching_aliases():
    userdata = SimpleNamespace(
        stt_text="I'd like a T BAG and some CHIPS, potato chips please.",
        objects_dict=OBJECTS,
        order_list=[],
    )
    spoken = []

    outcome = cb_state_check_order(userdata, None, spoken.append)

    assert outcome == "success"
    assert userdata.order_list == ["tea_bag", "potato_chips"]
    assert spoken == ["You ordered tea_bag, potato_chips."]


def test_check_order_retries_when_no_object_matches():
    userdata = SimpleNamespace(
        stt_text="water please", objects_dict=OBJECTS, order_list=[]
    )
    spoken = []

    outcome = cb_state_check_order(userdata, None, spoken.append)

    assert outcome == "timeout"
    assert userdata.order_list == []
    assert spoken == ["Sorry, I failed to understand your order. Please try again."]


def test_confirmation_selects_objects_by_canonical_name():
    userdata = SimpleNamespace(
        stt_text="YES",
        order_list=["tea_bag"],
        objects_dict=OBJECTS,
        request_objects_dict={},
    )
    spoken = []
    logger = SimpleNamespace(info=lambda _: None)
    node = SimpleNamespace(get_logger=lambda: logger)

    outcome = cb_state_check_order_confirmation(userdata, node, spoken.append)

    assert outcome == "apply"
    assert userdata.request_objects_dict == {"tea_bag": 0.1}


def test_order_report_uses_confirmed_object_names():
    userdata = SimpleNamespace(
        request_objects_dict={"tea_bag": 0.1, "potato_chips": 0.2},
        order_list=["t bag", "chips"],
    )
    spoken = []
    arm = SimpleNamespace(
        enable_upper_body_control=lambda _: None,
        joint_control=lambda **_: None,
        hand_control=lambda **_: None,
    )

    outcome = cb_state_order_report(userdata, None, spoken.append, arm)

    assert outcome == "success"
    assert spoken[0] == "Barman, the customer ordered tea_bag, potato_chips."
    assert userdata.order_list == []
