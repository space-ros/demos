import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3
import "qrc:/qml"

GridLayout {
    columns: 4
    columnSpacing: 10
    rowSpacing: 5  // Reduced from 10
    Layout.minimumWidth: 450
    Layout.minimumHeight: 450
    anchors.fill: parent
    anchors.margins: 10

    // Live Time
    ColumnLayout {
        id: liveTime
        Layout.columnSpan: 4
        Layout.fillWidth: true
        Layout.alignment: Qt.AlignCenter

        Label {
            text: DayLightManager.time
            font.pixelSize: 16
            font.bold: true
            Layout.alignment: Qt.AlignCenter
        }
    }

    // Planet Selection
    Label {
        text: "Planet:"
        font.pixelSize: 14
    }
    
    ComboBox {
        id: planetComboBoxId
        Layout.fillWidth: true
        Layout.columnSpan: 3
        model: ["Earth","Mars"]
        currentIndex: 0
        onCurrentIndexChanged: {
            DayLightManager.SetPlanet(model[currentIndex]);
        }

        ToolTip.visible: hovered
        ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
        ToolTip.text: qsTr("Select the planet to view the day light")
    }

    // Latitude
    Label { 
        text: "Latitude:"
        font.pixelSize: 14
    }

    TextField {
        id: latitudeId
        Layout.fillWidth: true
        Layout.columnSpan: 3
        text: "28.6139"
        validator: DoubleValidator {
            bottom: -90
            top: 90
            decimals: 2
            notation: DoubleValidator.StandardNotation
        }
        selectByMouse: true
        
        property string errorMessage: ""
        
        ToolTip.visible: hovered || errorMessage !== ""
        ToolTip.delay: hovered ? Qt.styleHints.mousePressAndHoldInterval : 0
        ToolTip.text: errorMessage !== "" ? errorMessage : qsTr("Enter the latitude of the location")

        onTextChanged: {
            var value = parseFloat(text)
            if (isNaN(value) || value < -90 || value > 90) {
                errorMessage = qsTr("Invalid latitude! Must be between -90 and 90.")
                color = "red"
            } else {
                errorMessage = ""
                color = "black"
                DayLightManager.SetLatitude(value);
            }
        }
    }

    // Set Time
    Label {
        text: "Set Time:"
        font.pixelSize: 14
    }

    TextField {
        id: timeTextId
        Layout.fillWidth: true
        Layout.columnSpan: 3
        text: "12"
        validator: IntValidator {
            bottom: 0
            top: 24
        }
        selectByMouse: true
        
        property string errorMessage: ""
        
        ToolTip.visible: hovered || errorMessage !== ""
        ToolTip.delay: hovered ? Qt.styleHints.mousePressAndHoldInterval : 0
        ToolTip.text: errorMessage !== "" ? errorMessage : qsTr("Enter time (0-24) hours")

        onTextChanged: {
            var value = parseInt(text)
            if (isNaN(value) || value < 0 || value > 24) {
                errorMessage = qsTr("Invalid Time of Day ! Must be between 0 and 24.")
                color = "red"
            } else {
                errorMessage = ""
                color = "black"
                DayLightManager.SetTime(value);
            }
        }
    }

    // Speed
    Label {
        text: "Speed:"
        font.pixelSize: 14
    }

    GzSpinBox { 
        id: speedSpinId
        Layout.fillWidth: true
        Layout.columnSpan: 3
        value: 1
        minimumValue: 1
        maximumValue: 100
        stepSize: 1      
        onEditingFinished: {
            DayLightManager.SetSpeed(speedSpinId.value);
        }
    }

    // X and Y coordinates
    Label {
        text: "X:"
        font.pixelSize: 14
    }

     GzSpinBox { 
        id: xSpinId
        value: 0
        minimumValue: -10000
        maximumValue: 10000
        stepSize: 1      
        onEditingFinished: {
            DayLightManager.SetX(xSpinId.value);
        }
    }

    Label {
    text: "Y:"
    font.pixelSize: 14
    Layout.alignment: Qt.AlignRight | Qt.AlignVCenter  // Align to the center vertically
}

GzSpinBox { 
    id: ySpinId
    value: 0
    minimumValue: -1000
    maximumValue: 10000
    stepSize: 1      
    Layout.alignment: Qt.AlignLeft | Qt.AlignVCenter  // Align to the center vertically
    onEditingFinished: {
        DayLightManager.SetY(ySpinId.value);
    }
}

    // Radius
    Label {
        text: "Radius:"
        font.pixelSize: 14
    }

    GzSpinBox {
        id: radiusSpinId
        value: 100
        minimumValue: 1
        maximumValue: 1000000  
        stepSize: 1
        onEditingFinished: {
            DayLightManager.SetRadius(radiusSpinId.value);
        }

        ToolTip.visible: hovered
        ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
        ToolTip.text: qsTr("Enter the solar path radius")
    }

    CheckBox {
        id: autoTimeProgressionId
        text: "Background Set"
        checked: false
        onCheckedChanged: {
            DayLightManager.BgColor(checked);
        }

        ToolTip.visible: hovered
        ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
        ToolTip.text: qsTr("Enable or disable background set")
    }

}