# DIGIT_CLASSIFICATION_V1



SVHN source: `C:\Users\slima\Downloads\SVHN number dataset.zip`

Printed source: `C:\Users\slima\Downloads\Printed-Digit-Detection.v2i.yolo26`

Printed YAML class mapping: `{0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6, 7: 7, 8: 8, 9: 9}`

SVHN MATLAB 7.3 HDF5 references decoded with h5py; label 10 maps to digit 0.

Printed YOLO boxes converted from normalized center coordinates to pixel boxes.

Source images: SVHN {'train': 33402, 'extra': 202353, 'test': 13068}; Printed {'train': 320, 'valid': 31, 'test': 22}.

Total crops: 357793. Splits: {'train': 325090, 'val': 6950, 'test': 25753}. Sources: {'SVHN': 357396, 'Printed': 397}.

Class counts: {0: 29003, 1: 61097, 2: 51429, 3: 41288, 4: 34610, 5: 35557, 6: 28147, 7: 29300, 8: 24143, 9: 23219}. Max/min nonempty class ratio: 2.63.

Source image resolution distribution: [((640, 640), 373), ((60, 27), 135), ((80, 35), 130), ((74, 34), 128), ((72, 33), 127), ((55, 26), 123), ((71, 33), 123), ((73, 33), 121), ((58, 27), 117), ((79, 35), 117), ((81, 35), 116), ((78, 35), 116), ((82, 35), 115), ((78, 34), 115), ((92, 42), 113), ((59, 28), 113), ((88, 40), 113), ((79, 34), 112), ((74, 33), 112), ((87, 40), 111)].

Crop resolution distribution: [((16, 30), 1561), ((15, 30), 1487), ((15, 26), 1446), ((16, 31), 1432), ((14, 26), 1419), ((15, 29), 1417), ((16, 29), 1414), ((17, 33), 1409), ((16, 32), 1387), ((15, 31), 1375), ((17, 32), 1364), ((20, 34), 1364), ((17, 31), 1343), ((17, 34), 1342), ((15, 32), 1335), ((14, 25), 1329), ((16, 33), 1325), ((14, 29), 1322), ((20, 33), 1303), ((16, 26), 1260)].

Smallest crop: (80, 'train/1/svhn_extra_135126_digit01.png').

Largest crop: (409600, 'train/8/printed_train_WIN_20240215_16_10_08_Pro_jpg_rf_eb10d330822a37273aeabe16b378846f_digit00.png').

Multi-digit source images: 134196.

Rejected crops: 4032; reasons: {'tiny_crop': 4032}.

Exact duplicate findings by original/new split: {('train', 'train'): 1}. Cross-split copies are excluded by test > val > train priority.

Missing files: [] (total 0). Annotation problems: [].

Near-duplicate perceptual hashing was not performed. Training has not started.

## Validation status and visual review

The complete image-by-image validation pass was stopped at the user's request because it was taking too long. The dataset is **not fully validated**. Build-time checks rejected invalid or tiny boxes, and each saved crop was encoded by Pillow, but the final all-file readability and folder reconciliation did not finish.

Visual review of the galleries found several Printed Digit crops with strokes cut off at image edges and some SVHN crops that include part of a neighboring digit. These examples need human review before model training; the current automatic filters do not detect all such semantic crop defects.
