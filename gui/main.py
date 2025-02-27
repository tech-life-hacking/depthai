# This Python file uses the following encoding: utf-8
import sys
from pathlib import Path

from PyQt5.QtQml import QQmlApplicationEngine, qmlRegisterType, qmlRegisterSingletonType, QQmlEngine
from PyQt5.QtQuick import QQuickPaintedItem
from PyQt5.QtGui import QImage
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot, QRunnable, QThreadPool
import depthai as dai

# To be used on the @QmlElement decorator
# (QML_IMPORT_MINOR_VERSION is optional)
from PyQt5.QtWidgets import QApplication
from depthai_sdk import Previews, resizeLetterbox


class Singleton(type(QQuickPaintedItem)):
    _instances = {}
    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


instance = None


# @QmlElement
class ImageWriter(QQuickPaintedItem):
    frame = QImage()

    def __init__(self, parent):
        super().__init__(parent)
        self.setRenderTarget(QQuickPaintedItem.FramebufferObject)
        self.setProperty("parent", parent)

    def paint(self, painter):
        painter.drawImage(0, 0, self.frame)

    def update_frame(self, image):
        self.frame = image
        self.update()


# @QmlElement
class AppBridge(QObject):
    @pyqtSlot()
    def applyAndRestart(self):
        instance.restartDemo()

    @pyqtSlot()
    def reloadDevices(self):
        instance.guiOnReloadDevices()

    @pyqtSlot(bool)
    def toggleStatisticsConsent(self, value):
        instance.guiOnStaticticsConsent(value)

    @pyqtSlot(str)
    def selectDevice(self, value):
        instance.guiOnSelectDevice(value)

    @pyqtSlot(bool, bool, bool)
    def selectReportingOptions(self, temp, cpu, memory):
        instance.guiOnSelectReportingOptions(temp, cpu, memory)

    @pyqtSlot(str)
    def selectReportingPath(self, value):
        instance.guiOnSelectReportingPath(value)

    @pyqtSlot(str)
    def selectEncodingPath(self, value):
        instance.guiOnSelectEncodingPath(value)

    @pyqtSlot(bool, int)
    def toggleColorEncoding(self, enabled, fps):
        instance.guiOnToggleColorEncoding(enabled, fps)

    @pyqtSlot(bool, int)
    def toggleLeftEncoding(self, enabled, fps):
        instance.guiOnToggleLeftEncoding(enabled, fps)

    @pyqtSlot(bool, int)
    def toggleRightEncoding(self, enabled, fps):
        instance.guiOnToggleRightEncoding(enabled, fps)

    @pyqtSlot(bool)
    def toggleDepth(self, enabled):
        instance.guiOnToggleDepth(enabled)

    @pyqtSlot(bool)
    def toggleNN(self, enabled):
        instance.guiOnToggleNN(enabled)

    @pyqtSlot(bool)
    def toggleDisparity(self, enabled):
        instance.guiOnToggleDisparity(enabled)


# @QmlElement
class AIBridge(QObject):
    @pyqtSlot(str)
    def setCnnModel(self, name):
        instance.guiOnAiSetupUpdate(cnn=name)

    @pyqtSlot(int)
    def setShaves(self, value):
        instance.guiOnAiSetupUpdate(shave=value)

    @pyqtSlot(str)
    def setModelSource(self, value):
        instance.guiOnAiSetupUpdate(source=value)

    @pyqtSlot(bool)
    def setFullFov(self, value):
        instance.guiOnAiSetupUpdate(fullFov=value)

    @pyqtSlot(bool)
    def setSbb(self, value):
        instance.guiOnAiSetupUpdate(sbb=value)

    @pyqtSlot(float)
    def setSbbFactor(self, value):
        if instance.writer is not None:
            instance.guiOnAiSetupUpdate(sbbFactor=value)

    @pyqtSlot(str)
    def setOvVersion(self, state):
        instance.guiOnAiSetupUpdate(ov=state.replace("VERSION_", ""))

    @pyqtSlot(str)
    def setCountLabel(self, state):
        instance.guiOnAiSetupUpdate(countLabel=state)


# @QmlElement
class PreviewBridge(QObject):
    @pyqtSlot(str)
    def changeSelected(self, state):
        instance.guiOnPreviewChangeSelected(state)


# @QmlElement
class DepthBridge(QObject):
    @pyqtSlot(bool)
    def toggleSubpixel(self, state):
        instance.guiOnDepthSetupUpdate(subpixel=state)

    @pyqtSlot(bool)
    def toggleExtendedDisparity(self, state):
        instance.guiOnDepthSetupUpdate(extended=state)

    @pyqtSlot(bool)
    def toggleLeftRightCheck(self, state):
        instance.guiOnDepthConfigUpdate(lrc=state)

    @pyqtSlot(int)
    def setDisparityConfidenceThreshold(self, value):
        instance.guiOnDepthConfigUpdate(dct=value)

    @pyqtSlot(int)
    def setLrcThreshold(self, value):
        instance.guiOnDepthConfigUpdate(lrcThreshold=value)

    @pyqtSlot(int)
    def setBilateralSigma(self, value):
        instance.guiOnDepthConfigUpdate(sigma=value)

    @pyqtSlot(int, int)
    def setDepthRange(self, valFrom, valTo):
        instance.guiOnDepthSetupUpdate(depthFrom=int(valFrom * 1000), depthTo=int(valTo * 1000))

    @pyqtSlot(str)
    def setMedianFilter(self, state):
        value = getattr(dai.MedianFilter, state)
        instance.guiOnDepthConfigUpdate(median=value)


# @QmlElement
class ColorCamBridge(QObject):
    name = "color"

    @pyqtSlot(int, int)
    def setIsoExposure(self, iso, exposure):
        if iso > 0 and exposure > 0:
            instance.guiOnCameraConfigUpdate("color", sensitivity=iso, exposure=exposure)

    @pyqtSlot(int)
    def setContrast(self, value):
        instance.guiOnCameraConfigUpdate("color", contrast=value)

    @pyqtSlot(int)
    def setBrightness(self, value):
        instance.guiOnCameraConfigUpdate("color", brightness=value)

    @pyqtSlot(int)
    def setSaturation(self, value):
        instance.guiOnCameraConfigUpdate("color", saturation=value)

    @pyqtSlot(int)
    def setSharpness(self, value):
        instance.guiOnCameraConfigUpdate("color", sharpness=value)

    @pyqtSlot(int)
    def setFps(self, value):
        instance.guiOnCameraSetupUpdate("color", fps=value)

    @pyqtSlot(str)
    def setResolution(self, state):
        if state == "THE_1080_P":
            instance.guiOnCameraSetupUpdate("color", resolution=1080)
        elif state == "THE_4_K":
            instance.guiOnCameraSetupUpdate("color", resolution=2160)
        elif state == "THE_12_MP":
            instance.guiOnCameraSetupUpdate("color", resolution=3040)


# @QmlElement
class MonoCamBridge(QObject):

    @pyqtSlot(int, int)
    def setIsoExposure(self, iso, exposure):
        if iso > 0 and exposure > 0:
            instance.guiOnCameraConfigUpdate("left", sensitivity=iso, exposure=exposure)
            instance.guiOnCameraConfigUpdate("right", sensitivity=iso, exposure=exposure)

    @pyqtSlot(int)
    def setContrast(self, value):
        instance.guiOnCameraConfigUpdate("left", contrast=value)
        instance.guiOnCameraConfigUpdate("right", contrast=value)

    @pyqtSlot(int)
    def setBrightness(self, value):
        instance.guiOnCameraConfigUpdate("left", brightness=value)
        instance.guiOnCameraConfigUpdate("right", brightness=value)

    @pyqtSlot(int)
    def setSaturation(self, value):
        instance.guiOnCameraConfigUpdate("left", saturation=value)
        instance.guiOnCameraConfigUpdate("right", saturation=value)

    @pyqtSlot(int)
    def setSharpness(self, value):
        instance.guiOnCameraConfigUpdate("left", sharpness=value)
        instance.guiOnCameraConfigUpdate("right", sharpness=value)

    @pyqtSlot(int)
    def setFps(self, value):
        instance.guiOnCameraSetupUpdate("left", fps=value)
        instance.guiOnCameraSetupUpdate("right", fps=value)

    @pyqtSlot(str)
    def setResolution(self, state):
        if state == "THE_720_P":
            instance.guiOnCameraSetupUpdate("left", resolution=720)
            instance.guiOnCameraSetupUpdate("right", resolution=720)
        elif state == "THE_800_P":
            instance.guiOnCameraSetupUpdate("left", resolution=800)
            instance.guiOnCameraSetupUpdate("right", resolution=800)
        elif state == "THE_400_P":
            instance.guiOnCameraSetupUpdate("left", resolution=400)
            instance.guiOnCameraSetupUpdate("right", resolution=400)


class DemoQtGui:
    instance = None
    writer = None
    window = None

    def __init__(self):
        global instance
        self.app = QApplication([sys.argv[0]])
        self.engine = QQmlApplicationEngine()
        self.engine.quit.connect(self.app.quit)
        instance = self
        qmlRegisterType(ImageWriter, 'dai.gui', 1, 0, 'ImageWriter')
        qmlRegisterType(AppBridge, 'dai.gui', 1, 0, 'AppBridge')
        qmlRegisterType(AIBridge, 'dai.gui', 1, 0, 'AIBridge')
        qmlRegisterType(PreviewBridge, 'dai.gui', 1, 0, 'PreviewBridge')
        qmlRegisterType(DepthBridge, 'dai.gui', 1, 0, 'DepthBridge')
        qmlRegisterType(ColorCamBridge, 'dai.gui', 1, 0, 'ColorCamBridge')
        qmlRegisterType(MonoCamBridge, 'dai.gui', 1, 0, 'MonoCamBridge')
        self.engine.addImportPath(str(Path(__file__).parent / "views"))
        self.engine.load(str(Path(__file__).parent / "views" / "root.qml"))
        self.window = self.engine.rootObjects()[0]
        if not self.engine.rootObjects():
            raise RuntimeError("Unable to start GUI - no root objects!")

    def setData(self, data):
        name, value = data
        self.window.setProperty(name, value)

    def updatePreview(self, frame):
        w, h = int(self.writer.width()), int(self.writer.height())
        scaledFrame = resizeLetterbox(frame, (w, h))
        if len(frame.shape) == 3:
            img = QImage(scaledFrame.data, w, h, frame.shape[2] * w, 29)  # 29 - QImage.Format_BGR888
        else:
            img = QImage(scaledFrame.data, w, h, w, 24)  # 24 - QImage.Format_Grayscale8
        self.writer.update_frame(img)

    def startGui(self):
        self.writer = self.window.findChild(QObject, "writer")
        medianChoices = list(filter(lambda name: name.startswith('KERNEL_') or name.startswith('MEDIAN_'), vars(dai.MedianFilter).keys()))[::-1]
        self.setData(["medianChoices", medianChoices])
        colorChoices = list(filter(lambda name: name[0].isupper(), vars(dai.ColorCameraProperties.SensorResolution).keys()))
        self.setData(["colorResolutionChoices", colorChoices])
        monoChoices = list(filter(lambda name: name[0].isupper(), vars(dai.MonoCameraProperties.SensorResolution).keys()))
        self.setData(["monoResolutionChoices", monoChoices])
        self.setData(["modelSourceChoices", [Previews.color.name, Previews.left.name, Previews.right.name]])
        versionChoices = sorted(filter(lambda name: name.startswith("VERSION_"), vars(dai.OpenVINO).keys()), reverse=True)
        self.setData(["ovVersions", versionChoices])
        return self.app.exec()
