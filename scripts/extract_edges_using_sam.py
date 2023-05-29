import argparse


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--image_path",
        dest="image_path",
        type=str,
        required=True,
        help="path to image file (.jpg or .png)",
    )
    parser.add_argument(
        "--output_json",
        dest="output_json",
        type=str,
        required=True,
        help="path to output json file",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    