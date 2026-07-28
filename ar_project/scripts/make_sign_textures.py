#!/usr/bin/env python3
"""Render the four sign-board textures used by the ar_project house world.

Standalone: PIL only, no ROS, no rclpy. Run it once, keep the PNGs in the repo,
re-run it whenever the wording changes. It is idempotent - every run overwrites
the same four files.

WHY THE TEXT IS ABSURDLY LARGE
------------------------------
The robot reads these boards with a 320x240 RGB camera whose optical centre is
at z = 0.13 m - ankle height. With horizontal_fov = 1.089 rad the pinhole focal
length is

    f = (320 / 2) / tan(1.089 / 2) = 264.6 px

A board is 1.60 m wide, so from a 2 m stand-off it projects to

    1.60 / 2.0 * 264.6 = ~212 px      out of the 320 px image width

The whole 1024 px texture is therefore resampled down to ~212 px: one texture
pixel survives as 0.207 image pixels. A VLM needs roughly 20 image pixels of
cap height to read a word reliably, which back-projects to

    20 / 0.207 = ~97 texture px = ~0.17 * 576 = about 1/5 of the board height
    (in the world: letters ~0.18 m tall)

Anything smaller - the "normal looking" 40-60 px caption you would put on a
poster - lands at 8-12 image pixels and is simply not there as far as the model
is concerned. Hence at most three or four glyph rows per board, each auto-sized
to eat its entire share of the vertical budget.

Consequences baked into the layout:

* Long strings are SPLIT across rows rather than shrunk to fit. Squeezing
  "КРУЖКА — НА КУХНЕ" onto one 944 px line forces a ~75 px font (55 px caps =
  11 image px at 2 m = unreadable); splitting it after the dash buys a ~150 px
  font and roughly doubles the distance at which the sign still works.
* Arrows are filled polygons, never glyphs, so they do not depend on font
  coverage and stay crisp under aggressive downsampling.
* The arrow occupies the TOP row and the words sit below it. The board spans
  z = 0.15 .. 1.05 m while the camera only sees up to z = 0.13 + 0.451*d, so at
  d = 1.5 m the top ~25 cm of the board is already out of frame. Keeping the
  room name low keeps it readable for longer as the robot closes in.

Board aspect: the physical board is 1.60 x 0.90 m = 16:9, so 1024 x 576 is a
pixel-exact match and the texture is never stretched on the mesh.
"""
import argparse
import math
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

# 1.60 m x 0.90 m board -> 16:9. Do not change one without the other.
TEX_W = 1024
TEX_H = 576
BOARD_W_M = 1.60
BOARD_H_M = 0.90

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

BORDER_PX = 14          # thick black frame, helps the VLM find the board edges
CONTENT_PAD_PX = 26     # quiet zone between the frame and the ink
ROW_GAP_PX = 14         # leading between stacked rows

FONT_PATH_DEFAULT = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
FONT_MIN_PX = 8
FONT_MAX_PX = 420

# Below this fraction of the board height a glyph row is worth warning about:
# 0.11 * 576 = 63 texture px = ~13 image px at 2 m, i.e. right at the edge of
# what the VLM can still resolve.
MIN_READABLE_FRACTION = 0.11

# Arrow proportions in the unit square, drawn pointing RIGHT and then remapped.
# HEAD_LEN is a fraction of the arrow length, the *_HALF values are fractions of
# the arrow thickness measured from the centre line.
ARROW_HEAD_LEN = 0.44
ARROW_SHAFT_HALF = 0.175
ARROW_HEAD_HALF = 0.50

# Final on-screen aspect (width / height) of the drawn arrow. Vertical arrows
# get a deliberately fat 0.85 instead of the natural 1/1.75, otherwise a rotated
# arrow ends up a skinny sliver that wastes the row it was given.
ARROW_ASPECT_H = 1.75
ARROW_ASPECT_V = 0.85

# Row weights are fractions of the content height and are normalised anyway, so
# they read as "share of the board this row deserves".
SIGNS = {
    'sign_cup_kitchen': [
        # "КРУЖКА — НА КУХНЕ" split after the dash: the trailing dash tells the
        # reader the phrase continues, and each half then gets a ~150 px font
        # instead of the ~75 px a single line would allow.
        {'kind': 'text', 'text': 'КРУЖКА —', 'weight': 0.34},
        {'kind': 'text', 'text': 'НА КУХНЕ', 'weight': 0.34},
        {'kind': 'text', 'text': 'CUP -> KITCHEN', 'weight': 0.32},
    ],
    # Wayfinding semantics matter more than they look. The robot reads these while
    # driving EAST down the hallway, so an arrow is interpreted relative to the image:
    # "->" means "further along that way", "^" means "straight on / in through here".
    # s5_arrow_signs therefore routes with two "^ ВАННАЯ" boards and uses "-> КУХНЯ"
    # as the distractor: the kitchen really IS further east, so the decoy carries
    # CORRECT information that is simply irrelevant to a towel mission. A factually
    # wrong sign would measure gullibility instead of goal-conditioned reading.
    'sign_arrow_right': [
        {'kind': 'arrow', 'direction': 'right', 'weight': 0.44},
        {'kind': 'text', 'text': 'КУХНЯ', 'weight': 0.30},
        {'kind': 'text', 'text': 'KITCHEN', 'weight': 0.26},
    ],
    # Mirror of the above; kept as a spare asset for user-authored scenarios where the
    # bathroom lies to the robot's image-left. No shipped scenario spawns it.
    'sign_arrow_left': [
        {'kind': 'arrow', 'direction': 'left', 'weight': 0.44},
        {'kind': 'text', 'text': 'ВАННАЯ', 'weight': 0.30},
        {'kind': 'text', 'text': 'BATHROOM', 'weight': 0.26},
    ],
    'sign_arrow_up': [
        # A vertical arrow needs more height than a horizontal one to read as an
        # arrow at all, so it takes half the board.
        {'kind': 'arrow', 'direction': 'up', 'weight': 0.50},
        {'kind': 'text', 'text': 'ВАННАЯ', 'weight': 0.28},
        {'kind': 'text', 'text': 'BATHROOM', 'weight': 0.22},
    ],
}


class FontBook:
    """Size -> font cache, plus the one-time fallback decision.

    The fit search asks for ~7 sizes per line, so caching keeps the whole run to
    a couple of dozen TrueType loads instead of a couple of hundred.
    """

    def __init__(self, font_path):
        self._path = font_path
        self._cache = {}
        self._truetype = Path(font_path).is_file()
        if not self._truetype:
            print('WARNING: font not found: %s' % font_path)
            print('WARNING: falling back to ImageFont.load_default() - the '
                  'glyphs will be small and the signs may be unreadable to the '
                  'VLM. Install fonts-dejavu-core to fix this.')

    @property
    def is_truetype(self):
        return self._truetype

    def get(self, size):
        font = self._cache.get(size)
        if font is not None:
            return font
        if self._truetype:
            font = ImageFont.truetype(self._path, size)
        else:
            try:
                # Pillow >= 10.1 can scale the bundled default face.
                font = ImageFont.load_default(size=size)
            except TypeError:
                font = ImageFont.load_default()
        self._cache[size] = font
        return font


def ink_box(font, text):
    """Ink bounding box of `text` relative to the default ("la") draw origin."""
    return font.getbbox(text)


def fit_text(fontbook, text, max_w, max_h):
    """Largest font size whose ink fits inside (max_w, max_h).

    Binary search rather than a fixed size: the Cyrillic and Latin lines have
    very different advance widths, and hand-tuning a size per string is exactly
    the kind of thing that silently starts clipping when the wording changes.
    """
    lo, hi = FONT_MIN_PX, FONT_MAX_PX
    best = None
    while lo <= hi:
        mid = (lo + hi) // 2
        font = fontbook.get(mid)
        box = ink_box(font, text)
        if (box[2] - box[0]) <= max_w and (box[3] - box[1]) <= max_h:
            best = (mid, font, box)
            lo = mid + 1
        else:
            hi = mid - 1
    if best is None:
        # Only reachable with the bitmap fallback font, which ignores `size`.
        font = fontbook.get(FONT_MIN_PX)
        best = (FONT_MIN_PX, font, ink_box(font, text))
        print('WARNING: "%s" does not fit in %dx%d even at the minimum size'
              % (text, max_w, max_h))
    return best


def draw_text_centered(draw, fontbook, text, box):
    """Draw `text` with its INK centred in `box`; returns the ink height."""
    x0, y0, x1, y1 = box
    size, font, ink = fit_text(fontbook, text, int(x1 - x0), int(y1 - y0))
    cx = (x0 + x1) / 2.0
    cy = (y0 + y1) / 2.0
    # font.getbbox() offsets are relative to the draw origin, so subtracting the
    # ink centre from the target centre centres the *painted pixels* rather than
    # the font's ascender/descender box. That matters here: an all-caps line has
    # a lot of dead space below the baseline and we cannot afford to waste it.
    ox = cx - (ink[0] + ink[2]) / 2.0
    oy = cy - (ink[1] + ink[3]) / 2.0
    draw.text((ox, oy), text, font=font, fill=BLACK)
    return size, (ink[3] - ink[1])


def arrow_polygon(direction):
    """Unit-square arrow polygon (x, y in [0, 1], y downwards)."""
    half_shaft = ARROW_SHAFT_HALF
    half_head = ARROW_HEAD_HALF
    base = 1.0 - ARROW_HEAD_LEN
    right = [
        (0.0, 0.5 - half_shaft),
        (base, 0.5 - half_shaft),
        (base, 0.5 - half_head),
        (1.0, 0.5),
        (base, 0.5 + half_head),
        (base, 0.5 + half_shaft),
        (0.0, 0.5 + half_shaft),
    ]
    if direction == 'right':
        return right
    if direction == 'left':
        return [(1.0 - x, y) for x, y in right]
    if direction == 'up':
        return [(y, 1.0 - x) for x, y in right]
    if direction == 'down':
        return [(1.0 - y, x) for x, y in right]
    raise ValueError('unknown arrow direction: %s' % direction)


def draw_arrow(draw, direction, box):
    """Draw a solid filled arrow, aspect-fitted and centred inside `box`.

    Deliberately a polygon and not a glyph: the sign has to look the same on any
    machine, and half the arrow codepoints are missing or differently shaped
    across font families.
    """
    x0, y0, x1, y1 = box
    bw = x1 - x0
    bh = y1 - y0
    aspect = ARROW_ASPECT_V if direction in ('up', 'down') else ARROW_ASPECT_H
    if bw / bh > aspect:
        h = bh
        w = bh * aspect
    else:
        w = bw
        h = bw / aspect
    ox = x0 + (bw - w) / 2.0
    oy = y0 + (bh - h) / 2.0
    pts = [(ox + ux * w, oy + uy * h) for ux, uy in arrow_polygon(direction)]
    draw.polygon(pts, fill=BLACK)
    return w, h


def layout_rows(rows, box, gap):
    """Split `box` vertically into one sub-box per row, by normalised weight."""
    x0, y0, x1, y1 = box
    total_weight = sum(r['weight'] for r in rows)
    usable = (y1 - y0) - gap * (len(rows) - 1)
    out = []
    cursor = float(y0)
    for row in rows:
        h = usable * (row['weight'] / total_weight)
        out.append((row, (float(x0), cursor, float(x1), cursor + h)))
        cursor += h + gap
    return out


def render_sign(name, rows, fontbook):
    img = Image.new('RGB', (TEX_W, TEX_H), WHITE)
    draw = ImageDraw.Draw(img)
    draw.rectangle([0, 0, TEX_W - 1, TEX_H - 1], outline=BLACK, width=BORDER_PX)

    inset = BORDER_PX + CONTENT_PAD_PX
    content = (inset, inset, TEX_W - inset, TEX_H - inset)

    report = []
    for row, box in layout_rows(rows, content, ROW_GAP_PX):
        if row['kind'] == 'text':
            size, ink_h = draw_text_centered(draw, fontbook, row['text'], box)
            frac = ink_h / float(TEX_H)
            report.append('    text  %-16s font=%3dpx ink_h=%3dpx (%.0f%% of '
                          'board, ~%.0f img px @2m)'
                          % ('"' + row['text'] + '"', size, ink_h, frac * 100.0,
                             ink_h * image_px_per_texture_px()))
            if frac < MIN_READABLE_FRACTION:
                print('WARNING: %s: line "%s" is only %.0f%% of the board '
                      'height - likely unreadable from 2 m.'
                      % (name, row['text'], frac * 100.0))
        elif row['kind'] == 'arrow':
            w, h = draw_arrow(draw, row['direction'], box)
            report.append('    arrow %-16s %dx%d px'
                          % (row['direction'], int(w), int(h)))
        else:
            raise ValueError('unknown row kind: %s' % row['kind'])
    return img, report


def image_px_per_texture_px(distance_m=2.0):
    """How many camera pixels one texture pixel survives as, at `distance_m`.

    Pure diagnostics - this is the number the module docstring argues about, so
    the run log prints it instead of making the reader redo the arithmetic.
    """
    focal_px = (320.0 / 2.0) / math.tan(1.089 / 2.0)
    board_px = BOARD_W_M / distance_m * focal_px
    return board_px / float(TEX_W)


def texture_path(out_root, name):
    return Path(out_root) / name / 'materials' / 'textures' / (name + '.png')


def parse_args(argv):
    default_root = Path(__file__).resolve().parent.parent / 'models'
    p = argparse.ArgumentParser(
        description='Render the ar_project sign-board textures (1024x576 PNG).')
    p.add_argument('--out-root', default=str(default_root),
                   help='models/ directory that holds <sign>/materials/textures'
                        ' (default: %(default)s)')
    p.add_argument('--font', default=FONT_PATH_DEFAULT,
                   help='bold TrueType face (default: %(default)s)')
    p.add_argument('--list', action='store_true', dest='list_only',
                   help='print what would be written and exit')
    return p.parse_args(argv)


def main(argv=None):
    # The sign copy is Cyrillic; a non-UTF-8 console must not kill the run.
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except (AttributeError, ValueError):
        pass

    args = parse_args(argv)
    out_root = Path(args.out_root).resolve()

    assert abs((TEX_W / float(TEX_H)) - (BOARD_W_M / BOARD_H_M)) < 1e-9, \
        'texture aspect must match the physical board'

    if args.list_only:
        print('out-root: %s' % out_root)
        print('scale:    1 texture px -> %.3f camera px at 2 m'
              % image_px_per_texture_px())
        for name, rows in SIGNS.items():
            print('%s -> %s' % (name, texture_path(out_root, name)))
            for row in rows:
                if row['kind'] == 'text':
                    print('    text  "%s"' % row['text'])
                else:
                    print('    arrow %s' % row['direction'])
        return 0

    fontbook = FontBook(args.font)
    print('font:     %s (%s)'
          % (args.font, 'truetype' if fontbook.is_truetype else 'FALLBACK'))
    print('out-root: %s' % out_root)
    print('scale:    1 texture px -> %.3f camera px at 2 m'
          % image_px_per_texture_px())

    for name, rows in SIGNS.items():
        img, report = render_sign(name, rows, fontbook)
        path = texture_path(out_root, name)
        # Only the textures directory is ours to create; model.sdf/model.config
        # in the same model folder belong to the model author.
        path.parent.mkdir(parents=True, exist_ok=True)
        img.save(path, format='PNG', optimize=True)
        print('%s  %dx%d  %d bytes' % (path, img.width, img.height,
                                       path.stat().st_size))
        for line in report:
            print(line)
    return 0


if __name__ == '__main__':
    sys.exit(main())
