from modulo_core import Predicate


def test_simple_predicate():
    predicate = Predicate(lambda: True)
    assert predicate.get_value()
    assert predicate.query()
    assert predicate.get_value()
    assert predicate.query() is None

    predicate.set_predicate(lambda: False)
    assert predicate.get_value() is False
    assert predicate.query() is False
    assert predicate.get_value() is False
    assert predicate.query() is None


def test_predicate_change_before_query():
    predicate = Predicate(lambda: False)
    predicate.set_predicate(lambda: True)
    assert predicate.get_value()
    assert predicate.query()
    assert predicate.query() is None
