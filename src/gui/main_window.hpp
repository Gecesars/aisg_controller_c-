#pragma once

#include "backend.hpp"

#include <QMainWindow>

#include <memory>
#include <future>
#include <optional>
#include <vector>

class QAction;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QFrame;
class QGroupBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QProgressBar;
class QPushButton;
class QSpinBox;
class QStackedWidget;
class QTableWidget;
class QTabWidget;
class QTimer;

namespace atc::gui {

class MainWindow final : public QMainWindow {
public:
    explicit MainWindow(std::unique_ptr<Backend> backend, QWidget* parent = nullptr);
    ~MainWindow() override;
    void runDemoScenario();

private:
    void buildMenus();
    void buildUi();
    QWidget* buildConnectionBar();
    QWidget* buildSummaryCards();
    QWidget* buildDevicePanel();
    QWidget* buildDetailsPanel();
    QWidget* buildOverviewTab();
    QWidget* buildMovementTab();
    QWidget* buildConfigurationTab();
    QWidget* buildDiagnosticsTab();
    QWidget* buildActivityPanel();
    void applyTheme();
    void restoreSettings();
    void saveSettings();

    void connectBackend();
    void disconnectBackend();
    void startScan();
    void finishScan();
    void cancelScan();
    void setConnectionState(bool connected);
    void updateActionState();
    void refreshTable();
    void refreshSummary();
    void refreshDetails();
    void refreshAlarms();
    void selectRowForAddress(std::uint8_t address);
    [[nodiscard]] std::optional<std::size_t> selectedDeviceIndex() const;
    [[nodiscard]] DeviceRecord* selectedDevice();
    [[nodiscard]] const DeviceRecord* selectedDevice() const;
    [[nodiscard]] bool filterAccepts(const DeviceRecord& device) const;

    void runRefresh();
    void runSelfTest();
    void runCalibrate();
    void runClearAlarms();
    void runSetTilt();
    void runMoveSector();
    void runSetGain();
    void runSetBypass();
    void applyConfiguration();
    void showManualAddressing();
    void showAntennaCatalog();
    void showSettingsDialog();
    void showAbout();
    void showUnsupportedFirmware();
    void exportSiteReport();
    void exportSessionLog();
    void appendLog(const QString& text, bool frame = false);
    void showResult(const OperationResult& result, const QString& operation);

    std::unique_ptr<Backend> backend_;
    std::vector<DeviceRecord> devices_;
    QTimer* scanTimer_{};
    int scanProgressValue_{};
    bool scanCancelled_{};
    std::future<std::vector<DeviceRecord>> scanFuture_;

    QComboBox* transportCombo_{};
    QComboBox* portCombo_{};
    QComboBox* baudCombo_{};
    QPushButton* connectButton_{};
    QPushButton* scanButton_{};
    QPushButton* cancelButton_{};
    QLabel* connectionBadge_{};
    QLabel* simulatorBanner_{};
    QProgressBar* scanProgress_{};

    QLabel* totalValue_{};
    QLabel* onlineValue_{};
    QLabel* retValue_{};
    QLabel* alarmValue_{};

    QLineEdit* searchEdit_{};
    QComboBox* filterCombo_{};
    QTableWidget* deviceTable_{};
    QPushButton* infoButton_{};
    QPushButton* alarmsButton_{};
    QPushButton* clearAlarmsButton_{};
    QPushButton* selfTestButton_{};

    QTabWidget* detailTabs_{};
    QLabel* emptyDetails_{};
    QLabel* deviceName_{};
    QLabel* deviceStatus_{};
    QLabel* uidValue_{};
    QLabel* productValue_{};
    QLabel* serialValue_{};
    QLabel* versionsValue_{};
    QLabel* stationValue_{};
    QLabel* sectorValue_{};
    QLabel* currentTiltValue_{};
    QLabel* limitsValue_{};
    QLabel* calibrationValue_{};
    QDoubleSpinBox* targetTiltSpin_{};
    QPushButton* moveButton_{};
    QPushButton* moveSectorButton_{};
    QDoubleSpinBox* gainSpin_{};
    QCheckBox* bypassCheck_{};
    QPushButton* applyTmaButton_{};

    QLineEdit* modelEdit_{};
    QLineEdit* antennaSerialEdit_{};
    QLineEdit* stationEdit_{};
    QLineEdit* sectorEdit_{};
    QLineEdit* dateEdit_{};
    QLineEdit* installerEdit_{};
    QLineEdit* technologyEdit_{};
    QLineEdit* locationEdit_{};
    QDoubleSpinBox* mechanicalTiltSpin_{};
    QDoubleSpinBox* bearingSpin_{};
    QDoubleSpinBox* heightSpin_{};
    QPushButton* applyConfigButton_{};
    QPushButton* calibrateButton_{};

    QTableWidget* alarmTable_{};
    QPlainTextEdit* eventLog_{};
    QPlainTextEdit* frameLog_{};
    QLabel* operationStatus_{};

    QAction* connectAction_{};
    QAction* disconnectAction_{};
    QAction* scanAction_{};
};

}  // namespace atc::gui
