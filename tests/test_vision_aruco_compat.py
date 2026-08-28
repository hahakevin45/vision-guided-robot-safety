from vgr_driver.vision import aruco


def test_aruco_detector_initializes_with_newer_opencv_aruco_api(monkeypatch):
    class FakeAruco:
        DICT_6X6_250 = 250

        @staticmethod
        def getPredefinedDictionary(dictionary_id):
            return ("dictionary", dictionary_id)

        @staticmethod
        def DetectorParameters():
            return "parameters"

    monkeypatch.setattr(aruco.cv2, "aruco", FakeAruco)

    from vgr_driver.vision import ArucoDetector
    detector = ArucoDetector()

    assert detector.dictionary == ("dictionary", 250)
    assert detector.parameters == "parameters"
