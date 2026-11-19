"""
フィールドのファイルから、地図の画像(.pgm)と設定(.yaml)を作るコマンド.

    ros2 run field_map make_map example                    # 地図をターミナルに出して確かめる
    ros2 run field_map make_map example -o ~/maps/field    # ~/maps/field.pgm と field.yaml を作る
                                                            # (位置合わせ用は field_lidar.pgm / .yaml)

作ったファイルは、Nav2 の map_server でも読める(field_map_node を使わないとき用)。
"""

import argparse
import sys

from . import field as fm


def main(argv=None):
    parser = argparse.ArgumentParser(description='フィールドのファイルから地図を作る')
    parser.add_argument('field', help='fields/ の中の名前(例: example)か、ファイルのパス')
    parser.add_argument('-o', '--output', help='書き出す先(拡張子なし)。なければ表示だけ')
    parser.add_argument('--columns', type=int, default=60, help='表示の横の文字数')
    args, _ = parser.parse_known_args(argv)     # ros2 run が付ける引数は無視する
    path = fm.resolve_field_path(args.field)
    try:
        field = fm.load_field(path)
    except fm.FieldError as e:
        sys.exit(str(e))
    cells = fm.rasterize(field, fm.NAV)
    lidar_cells = fm.rasterize(field, fm.LIDAR)
    print(f'{field.name}: x {field.x_min:g}〜{field.x_max:g} m、y {field.y_min:g}〜{field.y_max:g} m、'
          f'{field.width} × {field.height} マス、図形 {len(field.shapes)} 個({path})')
    print(fm.preview(field, cells, lidar_cells, args.columns))
    print(fm.PREVIEW_LEGEND)
    if args.output:
        written = fm.write_map_files(field, cells, args.output)
        written += fm.write_map_files(field, lidar_cells, args.output + '_lidar')
        for path_written in written:
            print(f'書きました: {path_written}')
