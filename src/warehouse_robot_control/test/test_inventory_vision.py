import cv2
import numpy as np

from warehouse_robot_control.inventory_vision import detect_boxes


def scene(upper, lower):
    image = np.full((360, 480, 3), 100, np.uint8)
    for y, colors in ((40, upper), (150, lower)):
        for i, color in enumerate(colors):
            rgb = (190, 25, 30) if color == 'red' else (20, 65, 190)
            cv2.rectangle(image, (60+i*65, y), (100+i*65, y+40), rgb, -1)
    return image


def test_counts_follow_image_content():
    for upper, lower in [(('blue', 'blue'), ('red', 'red', 'red')),
                         (('red', 'blue', 'blue'), ('blue', 'red')),
                         (('red',), ('blue', 'red', 'blue', 'blue'))]:
        result = detect_boxes(scene(upper, lower))
        assert result.valid, result.reason
        assert result.counts == (upper.count('red'), upper.count('blue'),
                                 lower.count('red'), lower.count('blue'))


def test_saturated_pixel_noise_does_not_add_boxes():
    image = scene(('blue', 'blue'), ('red', 'red', 'red'))
    rng = np.random.default_rng(17)
    for x, y in rng.integers((0, 0), (480, 360), size=(250, 2)):
        image[y, x] = (210, 10, 10)
    result = detect_boxes(image)
    assert result.valid and result.counts == (0, 2, 3, 0)


def test_clipped_box_invalidates_view():
    image = scene(('blue', 'blue'), ('red', 'red', 'red'))
    cv2.rectangle(image, (0, 150), (20, 190), (190, 25, 30), -1)
    assert not detect_boxes(image).valid


def test_partial_height_occlusion_is_rejected():
    image = scene(('blue', 'blue'), ('red', 'red', 'red'))
    image[150:180, 60:101] = 100
    assert not detect_boxes(image).valid


def test_touching_boxes_are_not_counted_as_one():
    image = scene(('blue',), ('red', 'red'))
    cv2.rectangle(image, (101, 150), (124, 190), (190, 25, 30), -1)
    assert not detect_boxes(image).valid


def test_one_level_or_blank_view_is_unreliable():
    assert not detect_boxes(scene(('blue', 'blue'), ())).valid
    assert not detect_boxes(np.zeros((360, 480, 3), np.uint8)).valid


def test_images_are_counted_independently():
    first = detect_boxes(scene(('blue', 'blue'), ('red', 'red', 'red')))
    for _ in range(20):
        assert detect_boxes(scene(('red',), ('blue',))).counts == (1, 0, 0, 1)
    assert first.counts == (0, 2, 3, 0)
