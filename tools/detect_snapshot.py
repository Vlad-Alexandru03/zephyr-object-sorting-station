import argparse
from pathlib import Path

import torch
from PIL import Image, ImageDraw, ImageFont
from torchvision.models.detection import (
    SSDLite320_MobileNet_V3_Large_Weights,
    ssdlite320_mobilenet_v3_large,
)


def load_detector():
    weights = SSDLite320_MobileNet_V3_Large_Weights.DEFAULT
    model = ssdlite320_mobilenet_v3_large(weights=weights).eval()
    return model, weights


def annotate(image, model, weights, threshold=0.5):
    categories = weights.meta["categories"]
    image_tensor = weights.transforms()(image)
    with torch.inference_mode():
        prediction = model([image_tensor])[0]

    draw = ImageDraw.Draw(image)
    font_size = max(14, round(min(image.size) * 0.03))
    font = ImageFont.load_default(size=font_size)
    padding = max(3, font_size // 5)
    detections = []

    for box, label, score in zip(
        prediction["boxes"], prediction["labels"], prediction["scores"]
    ):
        confidence = score.item()
        if confidence < threshold:
            continue

        category = categories[label.item()]
        x1, y1, x2, y2 = box.tolist()
        caption = f"{category} {confidence:.0%}"
        draw.rectangle((x1, y1, x2, y2), outline="lime", width=4)
        _, _, text_width, text_height = draw.textbbox(
            (0, 0), caption, font=font, anchor="lt"
        )
        label_width = text_width + 2 * padding
        label_height = text_height + 2 * padding
        label_x = max(0, min(round(x1), image.width - label_width))
        label_y = max(0, min(round(y1), image.height - label_height))
        draw.rectangle(
            (label_x, label_y, label_x + label_width, label_y + label_height),
            fill="black",
        )
        draw.text(
            (label_x + padding, label_y + padding),
            caption,
            font=font,
            fill="white",
            anchor="lt",
        )
        print(f"{caption}  box=({x1:.0f}, {y1:.0f}, {x2:.0f}, {y2:.0f})")
        detections.append((category, confidence))

    return detections


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("image", type=Path)
    parser.add_argument("--threshold", type=float, default=0.5)
    args = parser.parse_args()

    model, weights = load_detector()
    with Image.open(args.image) as source:
        image = source.convert("RGB")
    detections = annotate(image, model, weights, args.threshold)

    output = args.image.with_name(f"{args.image.stem}_detected.jpg")
    image.save(output, quality=95)
    print(f"Saved {output.resolve()} with {len(detections)} detections.")


if __name__ == "__main__":
    main()
