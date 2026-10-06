# Cube-1 camera pilot observations

These are three saved frames of the same numbered cube under changed orientation, not three independent test scenes. The visible glyph appears to be `1` in each capture. The `true_digit` column in `cube_crops.csv` remains blank until the owner confirms and labels the captures.

| Capture | YOLO cube | MobileNetV3-Small | ResNet18 | Observation |
| --- | --- | --- | --- | --- |
| `20261005_043045_030534` | 54% | 4 (96.3%) | 4 (96.1%) | Sideways glyph; both confidently wrong if true label is 1. |
| `20261005_043055_854856` | 73% | 1 (99.998%) | 1 (99.898%) | Upright glyph; both correct if true label is 1. |
| `20261005_043108_091606` | 75% | 6 (57.9%) | 2 (67.2%) | Inverted/angled glyph; models disagree and neither predicts 1. |

The saved `20261005_043052_045458` raw frame visibly contains the cube, but its labeled frame reports `Cubes: 0`; this is a YOLO miss, separate from digit recognition. Track IDs 10, 8, and 11 differ across captures, so do not treat them as permanent physical cube identities after a roll.

The cyan crop includes the printed face and some dark cube edge. It appears to contain the full `1`, so the repeated rotated errors are more consistent with an orientation/domain gap than a simple cut-off crop. The pilot is too small to estimate model accuracy or choose between MobileNet and ResNet.
