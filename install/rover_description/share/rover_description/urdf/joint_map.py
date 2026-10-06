#!/usr/bin/env python3
"""
Liczy GLOBALNE pozycje jointow wzgledem base_link.

Dump z samego <origin> pokazuje offset wzgledem RODZICA, wiec nie da sie
z niego odczytac gdzie kolo naprawde jest. Ten skrypt sklada transformacje
wzdluz calego lancucha.

    python3 joint_map.py /tmp/rover.urdf
"""

import sys
import math
import xml.etree.ElementTree as ET


def rpy_to_mat(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ]


def mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def apply(R, v):
    return [sum(R[i][k] * v[k] for k in range(3)) for i in range(3)]


def main(path):
    root = ET.parse(path).getroot()

    joints = {}
    children = {}
    for j in root.findall('joint'):
        name = j.get('name')
        parent = j.find('parent').get('link')
        child = j.find('child').get('link')
        o = j.find('origin')
        xyz = [float(v) for v in (o.get('xyz', '0 0 0').split()
                                  if o is not None else '0 0 0'.split())]
        rpy = [float(v) for v in (o.get('rpy', '0 0 0').split()
                                  if o is not None else '0 0 0'.split())]
        a = j.find('axis')
        axis = [float(v) for v in a.get('xyz').split()] if a is not None else None
        joints[name] = dict(type=j.get('type'), parent=parent, child=child,
                            xyz=xyz, rpy=rpy, axis=axis)
        children.setdefault(parent, []).append(name)

    rows = []

    def walk(link, pos, rot):
        for jn in children.get(link, []):
            j = joints[jn]
            p = [pos[i] + apply(rot, j['xyz'])[i] for i in range(3)]
            r = mul(rot, rpy_to_mat(*j['rpy']))
            if j['axis'] and j['type'] in ('revolute', 'continuous'):
                ax = apply(r, j['axis'])
                rows.append((jn, j['type'], p, ax, j['child']))
            walk(j['child'], p, r)

    base = 'base_link' if any(j['parent'] == 'base_link'
                              for j in joints.values()) else \
           next(iter(joints.values()))['parent']
    walk(base, [0, 0, 0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]])

    print(f"{'joint':14} {'typ':11} {'X(przod)':>9} {'Y(lewo)':>9} "
          f"{'Z':>8}   {'os globalna':18} rola")
    print('-' * 88)

    for name, typ, p, ax, child in sorted(rows, key=lambda r: -r[2][0]):
        # os pionowa -> skret, os pozioma poprzeczna -> naped
        vert = abs(ax[2])
        if vert > 0.9:
            rola = 'SKRET'
        elif abs(ax[1]) > 0.9 or abs(ax[0]) > 0.9:
            rola = 'KOLO' if 'felga' in child or 'wheel' in child.lower() else 'inne'
        else:
            rola = '?'

        side = 'L' if p[1] > 0.02 else ('P' if p[1] < -0.02 else '-')
        if rola in ('SKRET', 'KOLO'):
            rola += f'  {side}'

        print(f"{name:14} {typ:11} {p[0]:9.4f} {p[1]:9.4f} {p[2]:8.4f}   "
              f"[{ax[0]:5.2f} {ax[1]:5.2f} {ax[2]:5.2f}]  {rola}")

    print()
    wheels = [r for r in rows if 'felga' in r[4] or 'wheel' in r[4].lower()]
    if wheels:
        xs = sorted(set(round(w[2][0], 2) for w in wheels))
        ys = sorted(set(round(abs(w[2][1]), 2) for w in wheels))
        print(f'osie X kol: {xs}')
        print(f'|Y| kol:    {ys}')
        if len(xs) >= 3:
            a = (max(xs) - min(xs)) / 2
            print(f'\n-> a (os srodkowa -> przednia) = {a:.4f} m')
        if ys:
            print(f'-> b (polowa rozstawu)         = {max(ys):.4f} m')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else '/tmp/rover.urdf')