from __future__ import division

from refills_perception_interface.not_hacks import add_separator_between_barcodes, merge_close_separators


def test_add_separator_between_barcodes1():
    separators = [1, 2, 3]
    barcodes = [1.5, 2.5]
    separators, barcodes = add_separator_between_barcodes(separators, barcodes)
    assert len(separators) == 3
    assert len(barcodes) == 2


def test_add_separator_between_barcodes2():
    separators = [1, 3]
    barcodes = [1.5, 2.5]
    separators, barcodes = add_separator_between_barcodes(separators, barcodes)
    assert len(separators) == 3
    assert len(barcodes) == 2


def test_add_separator_between_barcodes3():
    separators = []
    barcodes = [1.5, 2.5]
    separators, barcodes = add_separator_between_barcodes(separators, barcodes)
    assert len(separators) == 1
    assert len(barcodes) == 2


def test_merge_close_separators1():
    separators = [0.01, 0.012]
    separators = merge_close_separators(separators)
    assert len(separators) == 1


def test_merge_close_separators2():
    separators = [0.0, 0.01, 0.012]
    separators = merge_close_separators(separators)
    assert len(separators) == 1


def test_merge_close_separators3():
    separators = [0.0, 0.01, 0.012, 0.1]
    separators = merge_close_separators(separators)
    assert len(separators) == 2


def test_merge_close_separators4():
    separators = [0.0, 0.01, 0.012, 0.1, 0.12, 0.2]
    separators = merge_close_separators(separators)
    assert len(separators) == 3


def test_merge_close_separators5():
    separators = [0.0, 0.057, 0.058, 0.1, 0.12, 0.2]
    separators = merge_close_separators(separators)
    assert len(separators) == 4
