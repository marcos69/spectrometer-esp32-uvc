#!/usr/bin/env python3
"""Validate jpegd (vendored libjpeg) against PIL on UVC-like test images."""
import numpy as np
from PIL import Image

OUT = '/tmp'
TOOL = '/home/ubuntu/workspace/spectrometer-esp32/build_test/jpegd_test'
import subprocess

def gen_spectrum():
    w, h = 640, 480
    im = np.zeros((h, w), dtype=float)
    im += 20 + 10 * np.linspace(1, 0.6, w)[None, :]
    for center, amp, width in ((150, 220, 3), (300, 180, 5), (420, 255, 2),
                               (500, 90, 40)):
        for x in range(w):
            d = (x - center) / width
            im[:, x] += amp * np.exp(-d * d)
    im += np.random.default_rng(7).normal(0, 2.5, im.shape)
    return np.clip(im, 0, 255).astype(np.uint8)

img = gen_spectrum()
rgb = np.stack([img, img, img], axis=2)
Image.fromarray(img, 'L').save(f'{OUT}/t_gray.jpg', 'JPEG', quality=80)
Image.fromarray(rgb, 'RGB').save(f'{OUT}/t_420.jpg', 'JPEG', quality=80, subsampling=2)
Image.fromarray(rgb, 'RGB').save(f'{OUT}/t_422.jpg', 'JPEG', quality=80, subsampling=1)

results = []
for name in ('t_gray', 't_420', 't_422'):
    r = subprocess.run([TOOL, f'{OUT}/{name}.jpg', f'{OUT}/jd_{name}.raw'],
                       capture_output=True, text=True)
    if r.returncode != 0:
        print(name, 'DECODE FAILED:', r.stderr)
        results.append(False)
        continue
    ref = np.array(Image.open(f'{OUT}/{name}.jpg').convert('L')).reshape(-1).astype(int)
    got = np.fromfile(f'{OUT}/jd_{name}.raw', dtype=np.uint8).astype(int)
    if got.shape != ref.shape:
        print(name, 'SIZE MISMATCH', got.shape, ref.shape)
        results.append(False)
        continue
    d = np.abs(got - ref)
    ok = d.max() <= 4
    results.append(ok)
    print(f'{name}: max={d.max():2d} mean={d.mean():5.2f} p99={np.percentile(d,99):4.1f} '
          f'{"OK" if ok else "FAIL"} (vs PIL)')

print('ALL PASS' if all(results) else 'FAILURES PRESENT')
