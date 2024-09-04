import QtQuick 2.9
import QtQuick.Controls 2.1
import QtQuick.Dialogs 1.0
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import gz.gui 1.0
import "qrc:/qml"

ColumnLayout {
    spacing: 20
    Layout.minimumWidth: 590
    Layout.minimumHeight: 590
    anchors.fill: parent
    anchors.margins: 20

    RowLayout {
        spacing: 20
        Layout.fillWidth: true

        Switch {
            Layout.alignment: Qt.AlignVCenter
            id: dustVisual
            Layout.columnSpan: 5
            Layout.fillWidth: true
            text: qsTr("Dust")
            checked: DustManager.emitting
            onToggled: {
                DustManager.ToggleDust(checked)
            }
            font.pixelSize: 16
        }
    }

    RowLayout {
        spacing: 20
        Layout.fillWidth: true

        Switch {
            Layout.alignment: Qt.AlignVCenter
            id: windVisual
            Layout.columnSpan: 5
            Layout.fillWidth: true
            text: qsTr("Wind")
            checked: false
            onToggled: {
                DustManager.ToggleWind(checked)
            }
            font.pixelSize: 16
        }

        RowLayout {
            spacing: 10
            Layout.fillWidth: true

            Label {
                Layout.columnSpan: 1
                text: "Color"
                font.pixelSize: 14
            }

            Button {
                Layout.columnSpan: 1
                id: minColorButton
                ToolTip.visible: hovered
                ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
                ToolTip.text: qsTr("Color for minimum value")
                onClicked: dustColorDialog.open()
                background: Rectangle {
                    implicitWidth: 40
                    implicitHeight: 40
                    radius: 5
                    border.color: DustManager.dustColorBorder
                    border.width: 2
                    color: DustManager.dustColor
                }
            }

            ColorDialog {
                id: dustColorDialog
                title: "Choose a color for the minimum value"
                visible: false
                onAccepted: {
                    DustManager.OnColorChanged(dustColorDialog.color)
                    dustColorDialog.close()
                }
                onRejected: {
                    dustColorDialog.close()
                }
            }
        }
    }

    GridLayout {
        columns: 3
        columnSpacing: 20
        rowSpacing: 15
        Layout.fillWidth: true

        Label {
            Layout.columnSpan: 1
            text: "Emitter Model"
            font.pixelSize: 14
        }

        ComboBox {
            Layout.columnSpan: 2
            id: emitterCombo
            Layout.fillWidth: true
            model: DustManager.dustModelList
            currentIndex: 0
            onCurrentIndexChanged: {
                if (currentIndex < 0)
                  return;
                DustManager.OnDustEmitterModel(textAt(currentIndex));
            }
            ToolTip.visible: hovered
            ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
            ToolTip.text: qsTr("Gazebo Transport topics publishing Dust messages")
        }
    }

    ColumnLayout {
        spacing: 15

        GridLayout {
            columns: 4
            columnSpacing: 10
            rowSpacing: 10

            Label {
                Layout.alignment: Qt.AlignVCenter
                text: "Density"
                font.pixelSize: 14
            }

            GzSpinBox {
                id: dustDensity
                Layout.fillWidth: true
                value: DustManager.density
                minimumValue: 1
                maximumValue: 1000
                decimals: 0
                onEditingFinished: {
                    DustManager.OnDensityChanged(dustDensity.value)
                }
            }

            Label {
                Layout.alignment: Qt.AlignVCenter
                text: "Angle"
                font.pixelSize: 14
            }
            
            GzSpinBox {
                id: angleSpin
                Layout.fillWidth: true
                value: DustManager.angle
                minimumValue: -3
                maximumValue: 3
                decimals: 0
                onEditingFinished: {
                    DustManager.OnPoseChanged(poseXSpin.value, poseYSpin.value, poseZSpin.value, angleSpin.value)
                }
            }

            Label {
                Layout.alignment: Qt.AlignVCenter
                text: "Velocity"
                font.pixelSize: 14
            }
            
            GzSpinBox {
                id: velocitySpin
                Layout.fillWidth: true
                value: DustManager.velocity
                minimumValue: 1
                maximumValue: 1000
                decimals: 0
                onEditingFinished: {
                    DustManager.OnVelocityChanged(velocitySpin.value)
                }
            }
        }

        ColumnLayout {
            spacing: 10

            Label {
                text: "Pose"
                font.pixelSize: 16
                font.bold: true
            }

            GridLayout {
                columns: 6
                columnSpacing: 10
                rowSpacing: 5

                Label { text: "X"; font.pixelSize: 14 }
                GzSpinBox { id: poseXSpin; value: DustManager.poseX; minimumValue: -1000; maximumValue: 1000; decimals: 0 
                    onEditingFinished: {
                        DustManager.OnPoseChanged(poseXSpin.value, poseYSpin.value, poseZSpin.value, angleSpin.value)
                    }
                }

                Label { text: "Y"; font.pixelSize: 14 }
                GzSpinBox { id: poseYSpin; value: DustManager.poseY; minimumValue: -1000; maximumValue: 1000; decimals: 0 
                    onEditingFinished: {
                        DustManager.OnPoseChanged(poseXSpin.value, poseYSpin.value, poseZSpin.value, angleSpin.value)
                    }
                }

                Label { text: "Z"; font.pixelSize: 14 }
                GzSpinBox { id: poseZSpin; value: DustManager.poseZ; minimumValue: -1000; maximumValue: 1000; decimals: 0 
                    onEditingFinished: {
                        DustManager.OnPoseChanged(poseXSpin.value, poseYSpin.value, poseZSpin.value, angleSpin.value)
                    }
                }   
            }
        }

        ColumnLayout {
            spacing: 10

            Label {
                text: "Volume"
                font.pixelSize: 16
                font.bold: true
            }

            GridLayout {
                columns: 6
                columnSpacing: 10
                rowSpacing: 5

                Label { text: "L"; font.pixelSize: 14 }
                GzSpinBox { id: volumeLSpin; value: DustManager.sizeX; minimumValue: 0; maximumValue: 100; decimals: 0 
                    onEditingFinished: {
                        DustManager.OnSizeChanged(volumeLSpin.value, volumeHSpin.value, volumeBSpin.value)
                    }
                }

                Label { text: "B"; font.pixelSize: 14 }
                GzSpinBox { id: volumeBSpin; value: DustManager.sizeX; minimumValue: 0; maximumValue: 100; decimals: 0 
                    onEditingFinished: {
                        DustManager.OnSizeChanged(volumeLSpin.value, volumeHSpin.value, volumeBSpin.value)
                    }
                }

                Label { text: "H"; font.pixelSize: 14 }
                GzSpinBox { id: volumeHSpin; value: DustManager.sizeZ; minimumValue: 0; maximumValue: 100; decimals: 0 
                    onEditingFinished: {
                        DustManager.OnSizeChanged(volumeLSpin.value, volumeHSpin.value, volumeBSpin.value)
                    }
                }
            }
        }
    }
}