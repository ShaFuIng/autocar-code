import unittest
from copy import deepcopy
from dimension_sources import consolidate


class DimensionSourcesTests(unittest.TestCase):
    def test_preserves_existing_unknowns_and_measurements(self):
        measure = {"rev": 6, "data": {"items": {
            "envelope_width": {"value": 32, "status": "measured"},
            "body_height": {"value": None, "status": "pending"},
        }, "notes": "original"}}
        original = deepcopy(measure)
        config = {"rev": 8, "data": {"width": 99, "overall_height": 100, "track_width": 46}}
        result, added = consolidate(measure, config)
        self.assertEqual(added, ["track_width"])
        self.assertEqual(result["items"]["envelope_width"]["value"], 32)
        self.assertIsNone(result["items"]["body_height"]["value"])
        self.assertEqual(measure, original)
        self.assertEqual(result["notes"], "original")

    def test_retains_provenance_and_is_idempotent(self):
        config = {"rev": 8, "data": {"track_width": 46, "line_width": 4.5}}
        result, added = consolidate({"rev": 6, "data": {}}, config)
        self.assertEqual(len(added), 2)
        self.assertEqual(result["items"]["line_width"]["source"]["revision"], 8)
        self.assertEqual(result["items"]["line_width"]["status"], "review")
        again, changed = consolidate({"rev": 7, "data": result}, config)
        self.assertEqual(changed, [])
        self.assertEqual(result, again)

    def test_unknown_zero_and_negative_are_not_adopted(self):
        data, added = consolidate({"data": {}}, {"rev": 1, "data": {
            "width": None, "line_width": 0, "track_width": -1, "deck_height": True,
        }})
        self.assertEqual(added, [])
        self.assertEqual(data["items"], {})


if __name__ == "__main__":
    unittest.main()
