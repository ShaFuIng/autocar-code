"""Existing image-space width prior shared by estimation and control.

No metric calibration; coefficients unchanged from standalone-line-v1.
Keep this module free of model/OpenCV dependencies for the control service.
"""
WIDTH_FIT = (3.0946106821106825, -335.43768768768786)


def prior_width(y):
    return max(20.0, WIDTH_FIT[0]*y+WIDTH_FIT[1])
