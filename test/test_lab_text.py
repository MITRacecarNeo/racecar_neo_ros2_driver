"""The lab apps and the Mapper keep the wording the text audit removed out of their pages."""

from pathlib import Path

import pytest

SCRIPTS = Path(__file__).resolve().parent.parent / 'scripts'
GONE = {
    'dashboards/teleop_dashboard/teleop.html': (
        'Web teleop', "'STOPPED'", "'TIMED OUT'", 'Driving zero', 'Is the detector running?',
        'detection(s)', 'then Send', 'WASD<br>', "'No battery reading'", "toast('Loaded.')",
        "'m/s2'", "'uT'",
    ),
    'dashboards/linefollow_dashboard/linefollow.html': (
        'Line follow', "'LINE FOUND'", "'LINE LOST'", 'Holding the last command',
        'Drag both thumbs', 'Low above high wraps', "toast('Loaded.')",
    ),
    'dashboards/wallfollow_dashboard/wallfollow.html': (
        'Wall follow', 'Distances are capped here', "label: 'speed'", "toast('Loaded.')",
    ),
    'dashboards/camlabel_dashboard/camlabel.html': (
        'Camera label', 'Spacebar also snaps', '>Prev<', 'None yet', 'Snap failed', 'Saved u',
        'Pick an unlabeled image', 'click a corner, move',
    ),
    'mapview.html': (
        'Checks are running', "'Checking...'", 'Saving...', 'Stopping...', 'Working...',
        'Failed: ', 'Use Keep in Maps', 'Speed limited for mapping', 'Where is the car',
        'Set pose: Tap', 'is open. Close it', 'Drive slowly until it shows', 'exists. Replace it?',
        'Cannot read', 'No maps yet', 'Mapping can start', "'done'",
    ),
    'web/logs.js': ("toast('Log saved.')",),
    'web/charts.js': ("'No data.'",),
}


@pytest.mark.parametrize('name', sorted(GONE))
def test_removed_wordings_stay_out(name):
    text = (SCRIPTS / name).read_text()
    for gone in GONE[name]:
        assert gone not in text, (name, gone)


@pytest.mark.parametrize(
    'name, labels',
    [
        ('dashboards/linefollow_dashboard/linefollow.html', ('Angle kp', 'Crop top', 'Min area')),
        ('dashboards/wallfollow_dashboard/wallfollow.html', ('Kp', 'Look angle', 'Side weight')),
        ('dashboards/teleop_dashboard/teleop.html', ("label: 'Speed'", "label: 'Angle'")),
    ],
)
def test_sliders_have_readable_labels(name, labels):
    text = (SCRIPTS / name).read_text()
    for label in labels:
        assert label in text, (name, label)
