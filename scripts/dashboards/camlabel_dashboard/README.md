# Camera Label Dashboard

Image capture and labeling as a service for the RACECAR Neo, on port 8085. Students capture frames from the car's camera, drag boxes on them, and the dashboard saves a YOLO-ready dataset for model training.

## Contents

- [Service control](#service-control)
- [Workflow](#workflow)
- [Settings](#settings)
- [Dataset](#dataset)
- [Training parameters](#training-parameters)

## Service control

Installed with the driver (`racecar setup dashboards`), stopped and disabled.
Open it from the Apps page on port 8080, or at `http://<car-ip>:8085`.

| Command | Does |
|---|---|
| `racecar service start camlabel` | start now |
| `racecar service stop camlabel` | stop |
| `racecar service enable camlabel` | start at every boot |
| `racecar service logs camlabel` | follow the log |

Camlabel only reads the camera, so it runs alongside a lab dashboard.

## Workflow

Collection and labeling are separate stages:

1. Collect: Snap (button or spacebar) saves the current full-resolution frame to the unlabeled pool. Take as many as you want, fast.
2. Label: click any thumbnail in the unlabeled section. Drag boxes on it, the class field names each box, Enter or Save labels finishes it. The image moves from unlabeled to labeled and the next unlabeled image loads automatically. Prev, Next, or the arrow keys move through the unlabeled images. Delete image throws a bad shot away, saving with no boxes makes a negative example.
3. Labeled images show as thumbnails with a click-to-view of their boxes.

## Settings

`camlabel.yaml` next to the script sets the live-preview resolution and jpeg quality sent to the browser (saved images are always the full camera frame). Restart the service after editing. Camera and preview resolutions show in the top bar.

## Dataset

Saved on the car in `datasets/camlabel/` under the data root (`racecar data path`), outside the driver, so updating or reinstalling the driver leaves it alone. The tree is created at startup when missing:

- `unlabeled/uN.jpg`: collected frames waiting for labels (not part of the training set)
- `images/imgN.jpg`: the frame as captured, full camera resolution JPEG, no boxes burned in
- `labels/imgN.txt`: YOLO format, one line per box: `class_id cx cy w h`, normalized 0..1
- `classes.txt`: one class name per line, id = line number
- `train_params.txt`: training parameters saved from the dashboard

This layout trains directly with ultralytics by pointing a data.yaml at it.

## Training parameters

The bottom card holds the common YOLO parameters (model, epochs, batch, image size, learning rate, splits, augmentations), each with a short description. Save params writes them to `train_params.txt` in that folder. A rough training time estimate for a Jetson Orin Nano updates live as values change.
