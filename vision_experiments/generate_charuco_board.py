"""Generate a print-size ChArUco SVG; measure the printed squares before calibration."""

import argparse
import base64
from pathlib import Path

import cv2


def positive_float(value):
    number = float(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return number


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--squares-x", type=int, required=True,
                        help="Number of chessboard squares across")
    parser.add_argument("--squares-y", type=int, required=True,
                        help="Number of chessboard squares down")
    parser.add_argument("--square-mm", type=positive_float, required=True,
                        help="Intended printed chessboard square side in mm")
    parser.add_argument("--marker-mm", type=positive_float, required=True,
                        help="Intended printed ArUco marker side in mm")
    parser.add_argument("--output", type=Path, default=Path("calibration/charuco_board.svg"))
    args = parser.parse_args()

    if args.squares_x < 2 or args.squares_y < 2:
        parser.error("both square counts must be at least 2")
    if args.marker_mm >= args.square_mm:
        parser.error("--marker-mm must be smaller than --square-mm")

    width_mm = args.squares_x * args.square_mm
    height_mm = args.squares_y * args.square_mm
    output = args.output.resolve()
    if output.exists():
        parser.error(f"output already exists: {output}")

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = cv2.aruco.CharucoBoard(
        (args.squares_x, args.squares_y),
        args.square_mm,
        args.marker_mm,
        dictionary,
    )
    pixels_per_mm = 12
    image = board.generateImage(
        (round(width_mm * pixels_per_mm), round(height_mm * pixels_per_mm)),
        marginSize=0,
    )
    ok, png = cv2.imencode(".png", image)
    if not ok:
        raise RuntimeError("OpenCV could not encode the board image")
    encoded = base64.b64encode(png.tobytes()).decode("ascii")
    svg = (
        f'<svg xmlns="http://www.w3.org/2000/svg" '
        f'xmlns:xlink="http://www.w3.org/1999/xlink" '
        f'width="{width_mm:g}mm" height="{height_mm:g}mm" '
        f'viewBox="0 0 {image.shape[1]} {image.shape[0]}">\n'
        f'<image width="{image.shape[1]}" height="{image.shape[0]}" '
        f'xlink:href="data:image/png;base64,{encoded}"/>\n'
        f'</svg>\n'
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(svg, encoding="utf-8")
    print(f"Saved: {output}")
    print(f"Board: {args.squares_x} x {args.squares_y} squares, "
          f"{width_mm:g} x {height_mm:g} mm intended size")
    print(f"Intended square: {args.square_mm:g} mm; "
          f"marker: {args.marker_mm:g} mm; dictionary: DICT_4X4_50")
    print("Print at 100% / actual size. Measure several printed squares and "
          "the board width/height with a ruler before calibration.")


if __name__ == "__main__":
    main()
