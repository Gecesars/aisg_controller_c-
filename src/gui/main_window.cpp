#include "main_window.hpp"

#include <QAbstractItemView>
#include <QAction>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMenuBar>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QRegularExpression>
#include <QRegularExpressionValidator>
#include <QSettings>
#include <QSplitter>
#include <QStatusBar>
#include <QTableWidget>
#include <QTabWidget>
#include <QTextStream>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <utility>

namespace atc::gui {
namespace {

QString text(const std::string& value) { return QString::fromStdString(value); }

QString kindName(DeviceKind kind) {
    switch (kind) {
    case DeviceKind::Ret: return QStringLiteral("RET");
    case DeviceKind::Tma: return QStringLiteral("TMA");
    case DeviceKind::Adb: return QStringLiteral("ADB");
    case DeviceKind::Sensor: return QStringLiteral("Sensor");
    case DeviceKind::Unknown: return QStringLiteral("Desconhecido");
    }
    return QStringLiteral("Desconhecido");
}

QString stateName(DeviceState state) {
    switch (state) {
    case DeviceState::Offline: return QStringLiteral("Offline");
    case DeviceState::Discovered: return QStringLiteral("Descoberto");
    case DeviceState::Operational: return QStringLiteral("Operacional");
    case DeviceState::Busy: return QStringLiteral("Em operação");
    case DeviceState::Alarm: return QStringLiteral("Com alarme");
    case DeviceState::Fault: return QStringLiteral("Falha");
    }
    return QStringLiteral("Indefinido");
}

QColor stateColor(DeviceState state) {
    switch (state) {
    case DeviceState::Operational: return QColor(QStringLiteral("#22c55e"));
    case DeviceState::Busy: return QColor(QStringLiteral("#38bdf8"));
    case DeviceState::Alarm: return QColor(QStringLiteral("#f59e0b"));
    case DeviceState::Fault: return QColor(QStringLiteral("#ef4444"));
    case DeviceState::Discovered: return QColor(QStringLiteral("#a78bfa"));
    case DeviceState::Offline: return QColor(QStringLiteral("#64748b"));
    }
    return QColor(QStringLiteral("#64748b"));
}

QTableWidgetItem* item(const QString& value) {
    auto* result = new QTableWidgetItem(value);
    result->setFlags(result->flags() & ~Qt::ItemIsEditable);
    return result;
}

QLabel* mutedLabel(const QString& value = {}) {
    auto* label = new QLabel(value);
    label->setProperty("muted", true);
    label->setWordWrap(true);
    return label;
}

QFrame* separator() {
    auto* line = new QFrame;
    line->setFrameShape(QFrame::HLine);
    line->setProperty("separator", true);
    return line;
}

}  // namespace

MainWindow::MainWindow(std::unique_ptr<Backend> backend, QWidget* parent)
    : QMainWindow(parent), backend_(std::move(backend)) {
    setWindowTitle(QStringLiteral("Antenna Tilt Controller • Linux"));
    resize(1480, 920);
    setMinimumSize(1120, 720);
    buildMenus();
    buildUi();
    applyTheme();
    restoreSettings();

    backend_->setLogCallback([this](const std::string& message, bool frame) {
        QMetaObject::invokeMethod(this, [this, message, frame] {
            appendLog(text(message), frame);
        }, Qt::QueuedConnection);
    });

    scanTimer_ = new QTimer(this);
    scanTimer_->setInterval(32);
    connect(scanTimer_, &QTimer::timeout, this, [this] {
        if (scanFuture_.valid() &&
            scanFuture_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            finishScan();
            return;
        }
        scanProgressValue_ = std::min(95, scanProgressValue_ + 2);
        scanProgress_->setValue(scanProgressValue_);
        operationStatus_->setText(tr("Sondando barramento… %1%").arg(scanProgressValue_));
    });

    appendLog(QStringLiteral("Aplicação iniciada. O simulador é seguro e não movimenta hardware."));
    setConnectionState(false);
}

MainWindow::~MainWindow() {
    saveSettings();
    backend_->cancelCurrent();
    if (scanFuture_.valid()) {
        scanFuture_.wait();
        try { (void)scanFuture_.get(); } catch (...) {}
    }
    backend_->disconnect();
}

void MainWindow::runDemoScenario() {
    connectBackend();
    startScan();
}

void MainWindow::buildMenus() {
    auto* fileMenu = menuBar()->addMenu(tr("&Arquivo"));
    auto* exportReport = fileMenu->addAction(tr("Exportar relatório do site…"));
    exportReport->setShortcut(QKeySequence::Save);
    connect(exportReport, &QAction::triggered, this, [this] { exportSiteReport(); });
    auto* exportLog = fileMenu->addAction(tr("Exportar log da sessão…"));
    connect(exportLog, &QAction::triggered, this, [this] { exportSessionLog(); });
    fileMenu->addSeparator();
    auto* exitAction = fileMenu->addAction(tr("Sair"));
    exitAction->setShortcut(QKeySequence::Quit);
    connect(exitAction, &QAction::triggered, this, &QWidget::close);

    auto* communicationMenu = menuBar()->addMenu(tr("&Comunicação"));
    connectAction_ = communicationMenu->addAction(tr("Conectar"));
    disconnectAction_ = communicationMenu->addAction(tr("Desconectar"));
    communicationMenu->addSeparator();
    scanAction_ = communicationMenu->addAction(tr("Localizar dispositivos"));
    connect(connectAction_, &QAction::triggered, this, [this] { connectBackend(); });
    connect(disconnectAction_, &QAction::triggered, this, [this] { disconnectBackend(); });
    connect(scanAction_, &QAction::triggered, this, [this] { startScan(); });

    auto* toolsMenu = menuBar()->addMenu(tr("&Ferramentas"));
    auto* catalogAction = toolsMenu->addAction(tr("Catálogo de antenas…"));
    auto* addressingAction = toolsMenu->addAction(tr("Endereçamento manual…"));
    auto* calibrationAction = toolsMenu->addAction(tr("Calibrar selecionado"));
    toolsMenu->addSeparator();
    auto* firmwareAction = toolsMenu->addAction(tr("Firmware / download…"));
    auto* settingsAction = toolsMenu->addAction(tr("Preferências…"));
    connect(catalogAction, &QAction::triggered, this, [this] { showAntennaCatalog(); });
    connect(addressingAction, &QAction::triggered, this, [this] { showManualAddressing(); });
    connect(calibrationAction, &QAction::triggered, this, [this] { runCalibrate(); });
    connect(firmwareAction, &QAction::triggered, this, [this] { showUnsupportedFirmware(); });
    connect(settingsAction, &QAction::triggered, this, [this] { showSettingsDialog(); });

    auto* helpMenu = menuBar()->addMenu(tr("A&juda"));
    auto* aboutAction = helpMenu->addAction(tr("Sobre o Antenna Tilt Controller"));
    connect(aboutAction, &QAction::triggered, this, [this] { showAbout(); });
}

void MainWindow::buildUi() {
    auto* central = new QWidget;
    auto* root = new QVBoxLayout(central);
    root->setContentsMargins(18, 14, 18, 16);
    root->setSpacing(12);

    auto* titleRow = new QHBoxLayout;
    auto* logo = new QLabel(QStringLiteral("AT"));
    logo->setObjectName(QStringLiteral("logoMark"));
    logo->setAlignment(Qt::AlignCenter);
    logo->setFixedSize(46, 46);
    auto* titleBox = new QVBoxLayout;
    auto* title = new QLabel(QStringLiteral("ANTENNA TILT CONTROLLER"));
    title->setObjectName(QStringLiteral("appTitle"));
    auto* subtitle = mutedLabel(QStringLiteral("Controle e diagnóstico AISG • C++ / Linux"));
    titleBox->addWidget(title);
    titleBox->addWidget(subtitle);
    titleRow->addWidget(logo);
    titleRow->addLayout(titleBox);
    titleRow->addStretch();
    simulatorBanner_ = new QLabel(QStringLiteral("● MODO SIMULADO"));
    simulatorBanner_->setObjectName(QStringLiteral("simulatorBanner"));
    titleRow->addWidget(simulatorBanner_);
    root->addLayout(titleRow);
    root->addWidget(buildConnectionBar());
    root->addWidget(buildSummaryCards());

    auto* mainSplitter = new QSplitter(Qt::Horizontal);
    mainSplitter->setChildrenCollapsible(false);
    mainSplitter->addWidget(buildDevicePanel());
    mainSplitter->addWidget(buildDetailsPanel());
    mainSplitter->setStretchFactor(0, 7);
    mainSplitter->setStretchFactor(1, 4);
    mainSplitter->setSizes({900, 500});
    root->addWidget(mainSplitter, 1);
    root->addWidget(buildActivityPanel());
    setCentralWidget(central);

    statusBar()->setSizeGripEnabled(false);
    operationStatus_ = new QLabel(QStringLiteral("Pronto"));
    statusBar()->addWidget(operationStatus_, 1);
    auto* safety = new QLabel(QStringLiteral("Firmware real bloqueado • comandos de movimento exigem confirmação"));
    safety->setProperty("muted", true);
    statusBar()->addPermanentWidget(safety);
}

QWidget* MainWindow::buildConnectionBar() {
    auto* panel = new QFrame;
    panel->setObjectName(QStringLiteral("connectionPanel"));
    auto* layout = new QHBoxLayout(panel);
    layout->setContentsMargins(14, 10, 14, 10);
    layout->setSpacing(10);

    layout->addWidget(mutedLabel(QStringLiteral("TRANSPORTE")));
    transportCombo_ = new QComboBox;
    transportCombo_->addItem(QStringLiteral("Simulador integrado"), true);
    transportCombo_->addItem(QStringLiteral("Serial RS-485 • AISG 3.0.8"), false);
    transportCombo_->setMinimumWidth(210);
    layout->addWidget(transportCombo_);

    layout->addWidget(mutedLabel(QStringLiteral("PORTA")));
    portCombo_ = new QComboBox;
    portCombo_->setEditable(true);
    QStringList endpoints{QStringLiteral("virtual://aisg")};
    const QDir byId(QStringLiteral("/dev/serial/by-id"));
    for (const auto& entry : byId.entryInfoList(QDir::Files | QDir::System | QDir::NoDotAndDotDot)) {
        endpoints.append(entry.absoluteFilePath());
    }
    const QDir dev(QStringLiteral("/dev"));
    for (const auto& name : dev.entryList({QStringLiteral("ttyUSB*"), QStringLiteral("ttyACM*"),
                                           QStringLiteral("ttyS*")}, QDir::System | QDir::Files)) {
        const auto endpoint = QStringLiteral("/dev/") + name;
        if (!endpoints.contains(endpoint)) endpoints.append(endpoint);
    }
    if (endpoints.size() == 1) {
        endpoints.append(QStringLiteral("/dev/ttyUSB0"));
        endpoints.append(QStringLiteral("/dev/ttyACM0"));
    }
    portCombo_->addItems(endpoints);
    portCombo_->setMinimumWidth(165);
    layout->addWidget(portCombo_);

    layout->addWidget(mutedLabel(QStringLiteral("BAUD")));
    baudCombo_ = new QComboBox;
    baudCombo_->addItems({QStringLiteral("9600"), QStringLiteral("38400"), QStringLiteral("115200")});
    baudCombo_->setCurrentText(QStringLiteral("9600"));
    layout->addWidget(baudCombo_);
    layout->addStretch();

    connectionBadge_ = new QLabel(QStringLiteral("● DESCONECTADO"));
    connectionBadge_->setObjectName(QStringLiteral("connectionBadge"));
    layout->addWidget(connectionBadge_);
    connectButton_ = new QPushButton(QStringLiteral("Conectar"));
    connectButton_->setObjectName(QStringLiteral("primaryButton"));
    connectButton_->setMinimumWidth(112);
    layout->addWidget(connectButton_);
    connect(connectButton_, &QPushButton::clicked, this, [this] {
        backend_->connected() ? disconnectBackend() : connectBackend();
    });
    connect(transportCombo_, &QComboBox::currentIndexChanged, this, [this] {
        const bool simulator = transportCombo_->currentData().toBool();
        portCombo_->setCurrentText(simulator ? QStringLiteral("virtual://aisg") : QStringLiteral("/dev/ttyUSB0"));
        if (!simulator) baudCombo_->setCurrentText(QStringLiteral("9600"));
        baudCombo_->setEnabled(simulator && !backend_->connected());
        simulatorBanner_->setVisible(simulator);
        appendLog(simulator ? QStringLiteral("Transporte selecionado: simulador integrado")
                            : QStringLiteral("Serial AISG Base 3.0.8 / ADB 3.1.7 selecionado (9600 8N1)"));
    });
    return panel;
}

QWidget* MainWindow::buildSummaryCards() {
    auto* container = new QWidget;
    auto* layout = new QHBoxLayout(container);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(10);

    const auto addCard = [&](const QString& label, const QString& accent, QLabel*& value) {
        auto* card = new QFrame;
        card->setObjectName(QStringLiteral("summaryCard"));
        card->setStyleSheet(QStringLiteral("QFrame#summaryCard { border-top: 3px solid %1; }").arg(accent));
        auto* cardLayout = new QHBoxLayout(card);
        cardLayout->setContentsMargins(14, 10, 14, 10);
        value = new QLabel(QStringLiteral("0"));
        value->setObjectName(QStringLiteral("summaryValue"));
        auto* labelWidget = mutedLabel(label);
        cardLayout->addWidget(value);
        cardLayout->addStretch();
        cardLayout->addWidget(labelWidget);
        layout->addWidget(card, 1);
    };
    addCard(QStringLiteral("DISPOSITIVOS"), QStringLiteral("#38bdf8"), totalValue_);
    addCard(QStringLiteral("ONLINE"), QStringLiteral("#22c55e"), onlineValue_);
    addCard(QStringLiteral("RET"), QStringLiteral("#a78bfa"), retValue_);
    addCard(QStringLiteral("ALARMES"), QStringLiteral("#f59e0b"), alarmValue_);
    return container;
}

QWidget* MainWindow::buildDevicePanel() {
    auto* panel = new QFrame;
    panel->setObjectName(QStringLiteral("surface"));
    auto* layout = new QVBoxLayout(panel);
    layout->setContentsMargins(14, 12, 14, 12);
    layout->setSpacing(9);

    auto* toolbar = new QHBoxLayout;
    auto* sectionTitle = new QLabel(QStringLiteral("Dispositivos AISG"));
    sectionTitle->setObjectName(QStringLiteral("sectionTitle"));
    toolbar->addWidget(sectionTitle);
    searchEdit_ = new QLineEdit;
    searchEdit_->setPlaceholderText(QStringLiteral("Buscar serial, site ou setor…"));
    searchEdit_->setClearButtonEnabled(true);
    searchEdit_->setMinimumWidth(220);
    toolbar->addWidget(searchEdit_, 1);
    filterCombo_ = new QComboBox;
    filterCombo_->addItems({QStringLiteral("Todos"), QStringLiteral("Somente RET"),
                            QStringLiteral("Somente TMA"), QStringLiteral("Com alarmes")});
    toolbar->addWidget(filterCombo_);
    scanButton_ = new QPushButton(QStringLiteral("Localizar dispositivos"));
    scanButton_->setObjectName(QStringLiteral("primaryButton"));
    toolbar->addWidget(scanButton_);
    cancelButton_ = new QPushButton(QStringLiteral("Cancelar"));
    cancelButton_->setVisible(false);
    toolbar->addWidget(cancelButton_);
    layout->addLayout(toolbar);

    scanProgress_ = new QProgressBar;
    scanProgress_->setTextVisible(false);
    scanProgress_->setFixedHeight(4);
    scanProgress_->setRange(0, 100);
    scanProgress_->setVisible(false);
    layout->addWidget(scanProgress_);

    deviceTable_ = new QTableWidget;
    deviceTable_->setColumnCount(10);
    deviceTable_->setHorizontalHeaderLabels({QStringLiteral("ADDR"), QStringLiteral("TIPO"),
        QStringLiteral("STATUS"), QStringLiteral("SERIAL"), QStringLiteral("ESTAÇÃO"),
        QStringLiteral("SETOR"), QStringLiteral("AISG"), QStringLiteral("E-TILT"),
        QStringLiteral("GANHO"), QStringLiteral("PRODUTO")});
    deviceTable_->setSelectionBehavior(QAbstractItemView::SelectRows);
    deviceTable_->setSelectionMode(QAbstractItemView::SingleSelection);
    deviceTable_->setAlternatingRowColors(true);
    deviceTable_->setShowGrid(false);
    deviceTable_->verticalHeader()->setVisible(false);
    deviceTable_->verticalHeader()->setDefaultSectionSize(36);
    deviceTable_->horizontalHeader()->setStretchLastSection(true);
    deviceTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    deviceTable_->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Stretch);
    deviceTable_->horizontalHeader()->setSectionResizeMode(4, QHeaderView::Stretch);
    layout->addWidget(deviceTable_, 1);

    auto* actions = new QHBoxLayout;
    infoButton_ = new QPushButton(QStringLiteral("Atualizar informações"));
    alarmsButton_ = new QPushButton(QStringLiteral("Consultar alarmes"));
    clearAlarmsButton_ = new QPushButton(QStringLiteral("Limpar alarmes"));
    selfTestButton_ = new QPushButton(QStringLiteral("Self-test"));
    actions->addWidget(infoButton_);
    actions->addWidget(alarmsButton_);
    actions->addWidget(clearAlarmsButton_);
    actions->addWidget(selfTestButton_);
    actions->addStretch();
    layout->addLayout(actions);

    connect(scanButton_, &QPushButton::clicked, this, [this] { startScan(); });
    connect(cancelButton_, &QPushButton::clicked, this, [this] { cancelScan(); });
    connect(searchEdit_, &QLineEdit::textChanged, this, [this] { refreshTable(); });
    connect(filterCombo_, &QComboBox::currentIndexChanged, this, [this] { refreshTable(); });
    connect(deviceTable_, &QTableWidget::itemSelectionChanged, this, [this] {
        refreshDetails();
        updateActionState();
    });
    connect(infoButton_, &QPushButton::clicked, this, [this] { runRefresh(); });
    connect(alarmsButton_, &QPushButton::clicked, this, [this] { refreshAlarms(); detailTabs_->setCurrentIndex(3); });
    connect(clearAlarmsButton_, &QPushButton::clicked, this, [this] { runClearAlarms(); });
    connect(selfTestButton_, &QPushButton::clicked, this, [this] { runSelfTest(); });
    return panel;
}

QWidget* MainWindow::buildDetailsPanel() {
    auto* panel = new QFrame;
    panel->setObjectName(QStringLiteral("surface"));
    auto* layout = new QVBoxLayout(panel);
    layout->setContentsMargins(14, 12, 14, 12);
    layout->setSpacing(8);

    auto* header = new QHBoxLayout;
    auto* heading = new QVBoxLayout;
    deviceName_ = new QLabel(QStringLiteral("Nenhum dispositivo selecionado"));
    deviceName_->setObjectName(QStringLiteral("sectionTitle"));
    deviceStatus_ = mutedLabel(QStringLiteral("Selecione uma linha para ver detalhes"));
    heading->addWidget(deviceName_);
    heading->addWidget(deviceStatus_);
    header->addLayout(heading, 1);
    layout->addLayout(header);

    detailTabs_ = new QTabWidget;
    detailTabs_->addTab(buildOverviewTab(), QStringLiteral("Visão geral"));
    detailTabs_->addTab(buildMovementTab(), QStringLiteral("Controle"));
    detailTabs_->addTab(buildConfigurationTab(), QStringLiteral("Configuração"));
    detailTabs_->addTab(buildDiagnosticsTab(), QStringLiteral("Diagnóstico"));
    layout->addWidget(detailTabs_, 1);
    return panel;
}

QWidget* MainWindow::buildOverviewTab() {
    auto* page = new QWidget;
    auto* form = new QFormLayout(page);
    form->setContentsMargins(8, 14, 8, 8);
    form->setVerticalSpacing(12);
    uidValue_ = new QLabel(QStringLiteral("—"));
    uidValue_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    productValue_ = new QLabel(QStringLiteral("—"));
    serialValue_ = new QLabel(QStringLiteral("—"));
    versionsValue_ = new QLabel(QStringLiteral("—"));
    stationValue_ = new QLabel(QStringLiteral("—"));
    sectorValue_ = new QLabel(QStringLiteral("—"));
    form->addRow(QStringLiteral("UID"), uidValue_);
    form->addRow(QStringLiteral("Produto"), productValue_);
    form->addRow(QStringLiteral("Serial"), serialValue_);
    form->addRow(QStringLiteral("Hardware / Software"), versionsValue_);
    form->addRow(QStringLiteral("Estação base"), stationValue_);
    form->addRow(QStringLiteral("Setor"), sectorValue_);
    form->addRow(separator());
    auto* note = mutedLabel(QStringLiteral("Serial real: perfil auditado AISG Base 3.0.8 / ADB 3.1.7. Operações fora do perfil validado permanecem bloqueadas."));
    form->addRow(note);
    return page;
}

QWidget* MainWindow::buildMovementTab() {
    auto* page = new QWidget;
    auto* layout = new QVBoxLayout(page);
    layout->setContentsMargins(8, 12, 8, 8);

    auto* retGroup = new QGroupBox(QStringLiteral("Remote Electrical Tilt"));
    auto* retForm = new QFormLayout(retGroup);
    currentTiltValue_ = new QLabel(QStringLiteral("—"));
    currentTiltValue_->setObjectName(QStringLiteral("largeMeasurement"));
    limitsValue_ = mutedLabel(QStringLiteral("—"));
    calibrationValue_ = mutedLabel(QStringLiteral("—"));
    targetTiltSpin_ = new QDoubleSpinBox;
    targetTiltSpin_->setRange(-99.9, 99.9);
    targetTiltSpin_->setDecimals(1);
    targetTiltSpin_->setSingleStep(0.1);
    targetTiltSpin_->setSuffix(QStringLiteral(" °"));
    moveButton_ = new QPushButton(QStringLiteral("Mover selecionado"));
    moveButton_->setObjectName(QStringLiteral("dangerButton"));
    moveSectorButton_ = new QPushButton(QStringLiteral("Mover setor…"));
    auto* moveRow = new QHBoxLayout;
    moveRow->addWidget(moveButton_);
    moveRow->addWidget(moveSectorButton_);
    retForm->addRow(QStringLiteral("Tilt atual"), currentTiltValue_);
    retForm->addRow(QStringLiteral("Limites"), limitsValue_);
    retForm->addRow(QStringLiteral("Calibração"), calibrationValue_);
    retForm->addRow(QStringLiteral("Novo tilt"), targetTiltSpin_);
    retForm->addRow(moveRow);
    layout->addWidget(retGroup);

    auto* tmaGroup = new QGroupBox(QStringLiteral("Tower Mounted Amplifier"));
    auto* tmaForm = new QFormLayout(tmaGroup);
    gainSpin_ = new QDoubleSpinBox;
    gainSpin_->setRange(0.0, 31.75);
    gainSpin_->setDecimals(2);
    gainSpin_->setSingleStep(0.25);
    gainSpin_->setSuffix(QStringLiteral(" dB"));
    bypassCheck_ = new QCheckBox(QStringLiteral("Ativar bypass"));
    applyTmaButton_ = new QPushButton(QStringLiteral("Aplicar ao TMA"));
    tmaForm->addRow(QStringLiteral("Ganho"), gainSpin_);
    tmaForm->addRow(QString(), bypassCheck_);
    tmaForm->addRow(applyTmaButton_);
    layout->addWidget(tmaGroup);
    layout->addStretch();

    connect(moveButton_, &QPushButton::clicked, this, [this] { runSetTilt(); });
    connect(moveSectorButton_, &QPushButton::clicked, this, [this] { runMoveSector(); });
    connect(applyTmaButton_, &QPushButton::clicked, this, [this] { runSetGain(); runSetBypass(); });
    return page;
}

QWidget* MainWindow::buildConfigurationTab() {
    auto* page = new QWidget;
    auto* form = new QFormLayout(page);
    form->setContentsMargins(8, 12, 8, 8);
    form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    modelEdit_ = new QLineEdit;
    antennaSerialEdit_ = new QLineEdit;
    antennaSerialEdit_->setMaxLength(17);
    stationEdit_ = new QLineEdit;
    sectorEdit_ = new QLineEdit;
    dateEdit_ = new QLineEdit;
    dateEdit_->setMaxLength(6);
    dateEdit_->setPlaceholderText(QStringLiteral("MMDDYY"));
    dateEdit_->setValidator(new QRegularExpressionValidator(QRegularExpression(QStringLiteral("[0-9]{0,6}")), dateEdit_));
    installerEdit_ = new QLineEdit;
    installerEdit_->setMaxLength(5);
    technologyEdit_ = new QLineEdit;
    locationEdit_ = new QLineEdit;
    mechanicalTiltSpin_ = new QDoubleSpinBox;
    mechanicalTiltSpin_->setRange(-30.0, 30.0);
    mechanicalTiltSpin_->setDecimals(1);
    mechanicalTiltSpin_->setSuffix(QStringLiteral(" °"));
    bearingSpin_ = new QDoubleSpinBox;
    bearingSpin_->setRange(0.0, 359.9);
    bearingSpin_->setDecimals(1);
    bearingSpin_->setSuffix(QStringLiteral(" °"));
    heightSpin_ = new QDoubleSpinBox;
    heightSpin_->setRange(0.0, 999.0);
    heightSpin_->setDecimals(1);
    heightSpin_->setSuffix(QStringLiteral(" m"));
    form->addRow(QStringLiteral("Modelo da antena"), modelEdit_);
    form->addRow(QStringLiteral("Serial da antena"), antennaSerialEdit_);
    form->addRow(QStringLiteral("Base Station ID"), stationEdit_);
    form->addRow(QStringLiteral("Setor"), sectorEdit_);
    form->addRow(QStringLiteral("Data de instalação"), dateEdit_);
    form->addRow(QStringLiteral("Instalador"), installerEdit_);
    form->addRow(QStringLiteral("Tecnologia"), technologyEdit_);
    form->addRow(QStringLiteral("Localização"), locationEdit_);
    form->addRow(QStringLiteral("Tilt mecânico"), mechanicalTiltSpin_);
    form->addRow(QStringLiteral("Bearing"), bearingSpin_);
    form->addRow(QStringLiteral("Altura"), heightSpin_);
    applyConfigButton_ = new QPushButton(QStringLiteral("Aplicar configuração"));
    applyConfigButton_->setObjectName(QStringLiteral("primaryButton"));
    form->addRow(applyConfigButton_);
    connect(applyConfigButton_, &QPushButton::clicked, this, [this] { applyConfiguration(); });
    return page;
}

QWidget* MainWindow::buildDiagnosticsTab() {
    auto* page = new QWidget;
    auto* layout = new QVBoxLayout(page);
    layout->setContentsMargins(8, 12, 8, 8);
    auto* hint = mutedLabel(QStringLiteral("Alarmes ativos do dispositivo selecionado. Operações de diagnóstico são serializadas para evitar comandos concorrentes no barramento half-duplex."));
    layout->addWidget(hint);
    alarmTable_ = new QTableWidget;
    alarmTable_->setColumnCount(3);
    alarmTable_->setHorizontalHeaderLabels({QStringLiteral("CÓDIGO"), QStringLiteral("SEVERIDADE"), QStringLiteral("DESCRIÇÃO")});
    alarmTable_->verticalHeader()->setVisible(false);
    alarmTable_->horizontalHeader()->setStretchLastSection(true);
    alarmTable_->setSelectionBehavior(QAbstractItemView::SelectRows);
    alarmTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    layout->addWidget(alarmTable_, 1);
    auto* actions = new QHBoxLayout;
    calibrateButton_ = new QPushButton(QStringLiteral("Calibrar RET"));
    auto* selfTest = new QPushButton(QStringLiteral("Executar self-test"));
    auto* clear = new QPushButton(QStringLiteral("Limpar alarmes"));
    actions->addWidget(calibrateButton_);
    actions->addWidget(selfTest);
    actions->addWidget(clear);
    layout->addLayout(actions);
    connect(calibrateButton_, &QPushButton::clicked, this, [this] { runCalibrate(); });
    connect(selfTest, &QPushButton::clicked, this, [this] { runSelfTest(); });
    connect(clear, &QPushButton::clicked, this, [this] { runClearAlarms(); });
    return page;
}

QWidget* MainWindow::buildActivityPanel() {
    auto* tabs = new QTabWidget;
    tabs->setObjectName(QStringLiteral("activityTabs"));
    tabs->setMinimumHeight(150);
    tabs->setMaximumHeight(250);
    eventLog_ = new QPlainTextEdit;
    eventLog_->setReadOnly(true);
    eventLog_->setMaximumBlockCount(3000);
    frameLog_ = new QPlainTextEdit;
    frameLog_->setReadOnly(true);
    frameLog_->setMaximumBlockCount(5000);
    eventLog_->setPlaceholderText(QStringLiteral("Eventos da sessão…"));
    frameLog_->setPlaceholderText(QStringLiteral("Quadros TX/RX aparecerão aqui…"));
    tabs->addTab(eventLog_, QStringLiteral("Atividade"));
    tabs->addTab(frameLog_, QStringLiteral("HDLC / AISG"));
    auto* wrapper = new QFrame;
    wrapper->setObjectName(QStringLiteral("surface"));
    auto* layout = new QVBoxLayout(wrapper);
    layout->setContentsMargins(8, 4, 8, 8);
    layout->addWidget(tabs);
    return wrapper;
}

void MainWindow::applyTheme() {
    qApp->setStyleSheet(QStringLiteral(R"QSS(
        * { font-family: "Inter", "Noto Sans", "DejaVu Sans"; font-size: 12px; }
        QMainWindow, QWidget { background: #0b1220; color: #dce7f5; }
        QMenuBar { background: #0b1220; color: #9fb0c7; padding: 3px 8px; }
        QMenuBar::item:selected, QMenu::item:selected { background: #17243a; color: white; }
        QMenu { background: #111b2d; border: 1px solid #27364d; padding: 6px; }
        QStatusBar { background: #08101d; color: #9fb0c7; border-top: 1px solid #1c2b40; }
        QLabel#appTitle { font-size: 18px; font-weight: 800; letter-spacing: 2px; color: #f7fbff; }
        QLabel#logoMark { background: #0ea5e9; color: #ffffff; font-size: 18px; font-weight: 900; border-radius: 12px; }
        QLabel#simulatorBanner { background: #0b3a31; color: #6ee7b7; border: 1px solid #176b57; border-radius: 10px; padding: 7px 12px; font-weight: 700; }
        QLabel#connectionBadge { color: #94a3b8; font-weight: 700; padding: 6px; }
        QLabel#sectionTitle { color: #f8fafc; font-size: 15px; font-weight: 700; }
        QLabel#summaryValue { color: white; font-size: 24px; font-weight: 800; }
        QLabel#largeMeasurement { color: #7dd3fc; font-size: 28px; font-weight: 800; }
        QLabel[muted="true"] { color: #8293aa; }
        QFrame#connectionPanel, QFrame#surface, QFrame#summaryCard {
            background: #111b2d; border: 1px solid #213149; border-radius: 10px;
        }
        QFrame[separator="true"] { color: #23344c; max-height: 1px; }
        QGroupBox { border: 1px solid #27384f; border-radius: 8px; margin-top: 11px; padding: 10px; color: #a9bad0; font-weight: 700; }
        QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
        QPushButton { background: #19283e; border: 1px solid #30445f; border-radius: 7px; padding: 7px 12px; color: #dbeafe; }
        QPushButton:hover { background: #233955; border-color: #4d6788; }
        QPushButton:pressed { background: #102137; }
        QPushButton:disabled { color: #526279; background: #111a29; border-color: #1c2a3d; }
        QPushButton#primaryButton { background: #0284c7; border-color: #0ea5e9; color: white; font-weight: 700; }
        QPushButton#primaryButton:hover { background: #0895d4; }
        QPushButton#dangerButton { background: #7f1d1d; border-color: #b91c1c; color: #fee2e2; font-weight: 700; }
        QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox {
            background: #0a1423; border: 1px solid #2b3d56; border-radius: 6px; padding: 6px 8px; selection-background-color: #0369a1;
        }
        QLineEdit:focus, QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus { border-color: #38bdf8; }
        QComboBox::drop-down { border: 0; width: 20px; }
        QTableWidget, QPlainTextEdit { background: #0a1423; alternate-background-color: #0e192a; border: 1px solid #20314a; border-radius: 6px; gridline-color: #1e2d43; }
        QTableWidget::item { padding: 5px; border-bottom: 1px solid #17263a; }
        QTableWidget::item:selected { background: #12385a; color: white; }
        QHeaderView::section { background: #142238; color: #8fa4bf; border: 0; border-right: 1px solid #243650; padding: 7px; font-size: 10px; font-weight: 700; }
        QTabWidget::pane { border: 1px solid #22334c; border-radius: 6px; top: -1px; background: #0e1829; }
        QTabBar::tab { background: #111c2e; color: #8295ae; border: 1px solid #22334c; padding: 7px 12px; }
        QTabBar::tab:selected { background: #172a43; color: #e0f2fe; border-bottom-color: #38bdf8; }
        QProgressBar { background: #172438; border: 0; border-radius: 2px; }
        QProgressBar::chunk { background: #38bdf8; border-radius: 2px; }
        QSplitter::handle { background: #0b1220; width: 8px; }
        QScrollBar:vertical { background: #0b1422; width: 10px; margin: 0; }
        QScrollBar::handle:vertical { background: #30445f; min-height: 24px; border-radius: 5px; }
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
        QToolTip { background: #17243a; color: white; border: 1px solid #385272; }
    )QSS"));
}

void MainWindow::restoreSettings() {
    QSettings settings;
    restoreGeometry(settings.value(QStringLiteral("window/geometry")).toByteArray());
    transportCombo_->setCurrentIndex(settings.value(QStringLiteral("connection/transport"), 0).toInt());
    portCombo_->setCurrentText(settings.value(QStringLiteral("connection/port"), QStringLiteral("virtual://aisg")).toString());
    baudCombo_->setCurrentText(settings.value(QStringLiteral("connection/baud"), QStringLiteral("9600")).toString());
}

void MainWindow::saveSettings() {
    QSettings settings;
    settings.setValue(QStringLiteral("window/geometry"), saveGeometry());
    settings.setValue(QStringLiteral("connection/transport"), transportCombo_->currentIndex());
    settings.setValue(QStringLiteral("connection/port"), portCombo_->currentText());
    settings.setValue(QStringLiteral("connection/baud"), baudCombo_->currentText());
}

void MainWindow::connectBackend() {
    if (backend_->connected()) return;
    const bool simulator = transportCombo_->currentData().toBool();
    if (!simulator) {
        if (QMessageBox::warning(this, tr("Perfil serial AISG 3"),
            tr("O perfil serial implementa descoberta e atribuição XID AISG Base 3.0.8, negociação e leitura ADB 3.1.7. Comandos de movimento e configuração permanecem bloqueados.\n\nValide o adaptador RS-485 com controle automático de direção antes do uso. Deseja abrir a porta selecionada?"),
            QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Cancel) != QMessageBox::Ok) return;
    }
    ConnectionSettings settings{
        .simulator = simulator,
        .port = portCombo_->currentText().toStdString(),
        .baudRate = baudCombo_->currentText().toInt(),
    };
    const auto result = backend_->connect(settings);
    showResult(result, tr("Conexão"));
    setConnectionState(result.ok);
}

void MainWindow::disconnectBackend() {
    cancelScan();
    if (scanFuture_.valid()) {
        scanFuture_.wait();
        try { (void)scanFuture_.get(); } catch (...) {}
        scanTimer_->stop();
        scanProgress_->setVisible(false);
        scanButton_->setVisible(true);
        cancelButton_->setVisible(false);
        cancelButton_->setEnabled(true);
    }
    backend_->disconnect();
    devices_.clear();
    refreshTable();
    refreshSummary();
    refreshDetails();
    setConnectionState(false);
    appendLog(QStringLiteral("Sessão encerrada"));
}

void MainWindow::startScan() {
    if (!backend_->connected()) {
        connectBackend();
        if (!backend_->connected()) return;
    }
    if (scanTimer_->isActive()) return;
    scanCancelled_ = false;
    scanProgressValue_ = 0;
    scanProgress_->setValue(0);
    scanProgress_->setVisible(true);
    scanButton_->setVisible(false);
    cancelButton_->setVisible(true);
    deviceTable_->setEnabled(false);
    appendLog(QStringLiteral("Iniciando sondagem HDLC dos endereços configurados…"));
    operationStatus_->setText(QStringLiteral("Descoberta em andamento"));
    scanFuture_ = std::async(std::launch::async, [this] { return backend_->scan(); });
    scanTimer_->start();
    updateActionState();
}

void MainWindow::finishScan() {
    scanTimer_->stop();
    std::vector<DeviceRecord> result;
    if (scanFuture_.valid()) {
        try {
            result = scanFuture_.get();
        } catch (const std::exception& exception) {
            appendLog(tr("Falha na descoberta: %1").arg(QString::fromUtf8(exception.what())));
        }
    }
    if (!scanCancelled_) {
        devices_ = std::move(result);
        refreshTable();
        refreshSummary();
        appendLog(tr("Descoberta concluída: %1 dispositivo(s)").arg(devices_.size()));
        operationStatus_->setText(tr("%1 dispositivo(s) no barramento").arg(devices_.size()));
        if (!devices_.empty()) deviceTable_->selectRow(0);
    }
    scanProgress_->setVisible(false);
    scanButton_->setVisible(true);
    cancelButton_->setVisible(false);
    cancelButton_->setEnabled(true);
    deviceTable_->setEnabled(true);
    updateActionState();
}

void MainWindow::cancelScan() {
    if (!scanTimer_ || !scanTimer_->isActive()) return;
    scanCancelled_ = true;
    backend_->cancelCurrent();
    appendLog(QStringLiteral("Descoberta cancelada pelo usuário"));
    operationStatus_->setText(QStringLiteral("Cancelando descoberta…"));
    cancelButton_->setEnabled(false);
    updateActionState();
}

void MainWindow::setConnectionState(bool connected) {
    connectButton_->setText(connected ? QStringLiteral("Desconectar") : QStringLiteral("Conectar"));
    connectionBadge_->setText(connected ? QStringLiteral("● CONECTADO") : QStringLiteral("● DESCONECTADO"));
    connectionBadge_->setStyleSheet(connected ? QStringLiteral("color: #4ade80; font-weight: 700;")
                                              : QStringLiteral("color: #94a3b8; font-weight: 700;"));
    transportCombo_->setEnabled(!connected);
    portCombo_->setEnabled(!connected);
    baudCombo_->setEnabled(!connected && transportCombo_->currentData().toBool());
    connectAction_->setEnabled(!connected);
    disconnectAction_->setEnabled(connected);
    scanAction_->setEnabled(connected);
    scanButton_->setEnabled(connected);
    updateActionState();
}

void MainWindow::updateActionState() {
    const auto* device = selectedDevice();
    const bool ready = backend_->connected() && device && !scanTimer_->isActive();
    infoButton_->setEnabled(ready);
    alarmsButton_->setEnabled(ready);
    clearAlarmsButton_->setEnabled(ready && device->activeAlarms > 0);
    selfTestButton_->setEnabled(ready && !device->aisg3);
    const bool isRet = ready && !device->aisg3 && device->kind == DeviceKind::Ret;
    const bool isTma = ready && !device->aisg3 && device->kind == DeviceKind::Tma;
    moveButton_->setEnabled(isRet && device->calibrated);
    moveSectorButton_->setEnabled(isRet);
    calibrateButton_->setEnabled(isRet);
    applyConfigButton_->setEnabled(ready && !device->aisg3);
    targetTiltSpin_->setEnabled(isRet);
    gainSpin_->setEnabled(isTma);
    bypassCheck_->setEnabled(isTma);
    applyTmaButton_->setEnabled(isTma);
}

void MainWindow::refreshTable() {
    std::optional<std::uint8_t> previous;
    if (const auto* selected = selectedDevice()) previous = selected->address;
    deviceTable_->setSortingEnabled(false);
    deviceTable_->setRowCount(0);
    for (const auto& device : devices_) {
        if (!filterAccepts(device)) continue;
        const int row = deviceTable_->rowCount();
        deviceTable_->insertRow(row);
        auto* address = item(QStringLiteral("0x%1").arg(device.address, 2, 16, QLatin1Char('0')).toUpper());
        address->setData(Qt::UserRole, static_cast<int>(device.address));
        deviceTable_->setItem(row, 0, address);
        deviceTable_->setItem(row, 1, item(kindName(device.kind)));
        auto* state = item(stateName(device.state));
        state->setForeground(stateColor(device.state));
        deviceTable_->setItem(row, 2, state);
        deviceTable_->setItem(row, 3, item(text(device.serial)));
        deviceTable_->setItem(row, 4, item(text(device.baseStation)));
        deviceTable_->setItem(row, 5, item(text(device.sector)));
        deviceTable_->setItem(row, 6, item(text(device.aisgVersion)));
        deviceTable_->setItem(row, 7, item(device.kind == DeviceKind::Ret ? QStringLiteral("%1°").arg(device.electricalTilt, 0, 'f', 1) : QStringLiteral("—")));
        deviceTable_->setItem(row, 8, item(device.kind == DeviceKind::Tma ? QStringLiteral("%1 dB").arg(device.gain, 0, 'f', 2) : QStringLiteral("—")));
        deviceTable_->setItem(row, 9, item(text(device.product)));
    }
    if (previous) selectRowForAddress(*previous);
    updateActionState();
}

void MainWindow::refreshSummary() {
    totalValue_->setNum(static_cast<int>(devices_.size()));
    onlineValue_->setNum(static_cast<int>(std::count_if(devices_.begin(), devices_.end(), [](const auto& d) {
        return d.state != DeviceState::Offline && d.state != DeviceState::Fault;
    })));
    retValue_->setNum(static_cast<int>(std::count_if(devices_.begin(), devices_.end(), [](const auto& d) { return d.kind == DeviceKind::Ret; })));
    int alarms = 0;
    for (const auto& device : devices_) alarms += device.activeAlarms;
    alarmValue_->setNum(alarms);
}

void MainWindow::refreshDetails() {
    const auto* device = selectedDevice();
    if (!device) {
        deviceName_->setText(QStringLiteral("Nenhum dispositivo selecionado"));
        deviceStatus_->setText(QStringLiteral("Selecione uma linha para ver detalhes"));
        uidValue_->setText(QStringLiteral("—"));
        productValue_->setText(QStringLiteral("—"));
        serialValue_->setText(QStringLiteral("—"));
        versionsValue_->setText(QStringLiteral("—"));
        stationValue_->setText(QStringLiteral("—"));
        sectorValue_->setText(QStringLiteral("—"));
        refreshAlarms();
        updateActionState();
        return;
    }
    deviceName_->setText(QStringLiteral("%1 • endereço 0x%2").arg(kindName(device->kind)).arg(device->address, 2, 16, QLatin1Char('0')).toUpper());
    deviceStatus_->setText(stateName(device->state) + QStringLiteral(" • ") + text(device->vendor));
    deviceStatus_->setStyleSheet(QStringLiteral("color: %1;").arg(stateColor(device->state).name()));
    uidValue_->setText(text(device->uid));
    productValue_->setText(text(device->product));
    serialValue_->setText(text(device->serial));
    versionsValue_->setText(tr("HW %1  /  SW %2").arg(text(device->hardwareVersion), text(device->softwareVersion)));
    stationValue_->setText(text(device->baseStation));
    sectorValue_->setText(text(device->sector));
    currentTiltValue_->setText(device->kind == DeviceKind::Ret ? QStringLiteral("%1°").arg(device->electricalTilt, 0, 'f', 1) : QStringLiteral("—"));
    limitsValue_->setText(device->kind == DeviceKind::Ret ? tr("%1° a %2°").arg(device->minimumTilt, 0, 'f', 1).arg(device->maximumTilt, 0, 'f', 1) : QStringLiteral("Não aplicável"));
    calibrationValue_->setText(device->calibrated ? QStringLiteral("Calibrado") : QStringLiteral("Calibração necessária"));
    targetTiltSpin_->setRange(device->minimumTilt, device->maximumTilt);
    targetTiltSpin_->setValue(device->electricalTilt);
    gainSpin_->setValue(device->gain);
    bypassCheck_->setChecked(device->bypass);
    modelEdit_->setText(text(device->antennaModel));
    antennaSerialEdit_->setText(text(device->serial));
    stationEdit_->setText(text(device->baseStation));
    sectorEdit_->setText(text(device->sector));
    dateEdit_->setText(text(device->installationDate));
    installerEdit_->setText(text(device->installerId));
    technologyEdit_->setText(text(device->technology));
    locationEdit_->setText(text(device->location));
    mechanicalTiltSpin_->setValue(device->mechanicalTilt);
    bearingSpin_->setValue(device->bearing);
    heightSpin_->setValue(device->height);
    refreshAlarms();
    updateActionState();
}

void MainWindow::refreshAlarms() {
    alarmTable_->setRowCount(0);
    const auto* device = selectedDevice();
    if (!device || device->activeAlarms <= 0) return;
    alarmTable_->insertRow(0);
    alarmTable_->setItem(0, 0, item(QStringLiteral("0x0005")));
    alarmTable_->setItem(0, 1, item(QStringLiteral("Aviso")));
    alarmTable_->setItem(0, 2, item(QStringLiteral("Movimento interrompido anteriormente (simulado)")));
}

void MainWindow::selectRowForAddress(std::uint8_t address) {
    for (int row = 0; row < deviceTable_->rowCount(); ++row) {
        if (deviceTable_->item(row, 0)->data(Qt::UserRole).toInt() == address) {
            deviceTable_->selectRow(row);
            return;
        }
    }
}

std::optional<std::size_t> MainWindow::selectedDeviceIndex() const {
    const auto items = deviceTable_->selectedItems();
    if (items.empty()) return std::nullopt;
    const int row = items.front()->row();
    const auto* addressItem = deviceTable_->item(row, 0);
    if (!addressItem) return std::nullopt;
    const auto address = static_cast<std::uint8_t>(addressItem->data(Qt::UserRole).toUInt());
    for (std::size_t index = 0; index < devices_.size(); ++index) {
        if (devices_[index].address == address) return index;
    }
    return std::nullopt;
}

DeviceRecord* MainWindow::selectedDevice() {
    const auto index = selectedDeviceIndex();
    return index ? &devices_[*index] : nullptr;
}

const DeviceRecord* MainWindow::selectedDevice() const {
    const auto index = selectedDeviceIndex();
    return index ? &devices_[*index] : nullptr;
}

bool MainWindow::filterAccepts(const DeviceRecord& device) const {
    const int filter = filterCombo_->currentIndex();
    if (filter == 1 && device.kind != DeviceKind::Ret) return false;
    if (filter == 2 && device.kind != DeviceKind::Tma) return false;
    if (filter == 3 && device.activeAlarms == 0) return false;
    const auto needle = searchEdit_->text().trimmed();
    if (needle.isEmpty()) return true;
    const QString haystack = text(device.serial + " " + device.uid + " " + device.baseStation + " " + device.sector + " " + device.product);
    return haystack.contains(needle, Qt::CaseInsensitive);
}

void MainWindow::runRefresh() {
    auto* device = selectedDevice();
    if (!device) return;
    showResult(backend_->refresh(*device), tr("Atualização"));
    refreshTable();
    refreshDetails();
}

void MainWindow::runSelfTest() {
    auto* device = selectedDevice();
    if (!device) return;
    showResult(backend_->selfTest(*device), tr("Self-test"));
}

void MainWindow::runCalibrate() {
    auto* device = selectedDevice();
    if (!device) return;
    if (QMessageBox::question(this, tr("Confirmar calibração"),
        tr("Calibrar o RET %1 no endereço 0x%2?").arg(text(device->serial)).arg(device->address, 2, 16, QLatin1Char('0'))) != QMessageBox::Yes) return;
    showResult(backend_->calibrate(*device), tr("Calibração"));
    refreshDetails();
}

void MainWindow::runClearAlarms() {
    auto* device = selectedDevice();
    if (!device) return;
    if (QMessageBox::question(this, tr("Limpar alarmes"), tr("Confirmar a limpeza dos alarmes ativos?")) != QMessageBox::Yes) return;
    showResult(backend_->clearAlarms(*device), tr("Alarmes"));
    refreshTable();
    refreshSummary();
    refreshDetails();
}

void MainWindow::runSetTilt() {
    auto* device = selectedDevice();
    if (!device || device->kind != DeviceKind::Ret) return;
    const double target = targetTiltSpin_->value();
    const auto confirmation = tr("Mover o RET %1?\n\nTilt atual: %2°\nNovo tilt: %3°\nLimites: %4° a %5°\n\nO movimento será registrado no log da sessão.")
        .arg(text(device->serial)).arg(device->electricalTilt, 0, 'f', 1).arg(target, 0, 'f', 1)
        .arg(device->minimumTilt, 0, 'f', 1).arg(device->maximumTilt, 0, 'f', 1);
    if (QMessageBox::warning(this, tr("Confirmar movimento"), confirmation, QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel) != QMessageBox::Yes) return;
    showResult(backend_->setTilt(*device, target), tr("Movimento RET"));
    refreshTable();
    refreshDetails();
}

void MainWindow::runMoveSector() {
    auto* primary = selectedDevice();
    if (!primary || primary->kind != DeviceKind::Ret) return;
    const double target = targetTiltSpin_->value();
    std::vector<std::size_t> targets;
    for (std::size_t i = 0; i < devices_.size(); ++i) {
        if (devices_[i].kind == DeviceKind::Ret && devices_[i].sector == primary->sector &&
            target >= devices_[i].minimumTilt && target <= devices_[i].maximumTilt) targets.push_back(i);
    }
    if (targets.empty()) return;
    if (QMessageBox::warning(this, tr("Mover setor"),
        tr("Aplicar %1° sequencialmente a %2 RET(s) do setor %3?\n\nA operação não é atômica; uma falha pode produzir sucesso parcial.")
            .arg(target, 0, 'f', 1).arg(targets.size()).arg(text(primary->sector)),
        QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel) != QMessageBox::Yes) return;
    int success = 0;
    for (const auto index : targets) if (backend_->setTilt(devices_[index], target).ok) ++success;
    appendLog(tr("Movimento de setor concluído: %1/%2 RET(s)").arg(success).arg(targets.size()));
    QMessageBox::information(this, tr("Movimento de setor"), tr("%1 de %2 dispositivos concluídos.").arg(success).arg(targets.size()));
    refreshTable();
    refreshDetails();
}

void MainWindow::runSetGain() {
    auto* device = selectedDevice();
    if (!device || device->kind != DeviceKind::Tma) return;
    const double target = gainSpin_->value();
    if (QMessageBox::question(this, tr("Alterar ganho"), tr("Aplicar ganho de %1 dB ao TMA %2?").arg(target, 0, 'f', 2).arg(text(device->serial))) != QMessageBox::Yes) return;
    showResult(backend_->setGain(*device, target), tr("Ganho TMA"));
    refreshTable();
}

void MainWindow::runSetBypass() {
    auto* device = selectedDevice();
    if (!device || device->kind != DeviceKind::Tma) return;
    showResult(backend_->setBypass(*device, bypassCheck_->isChecked()), tr("Modo TMA"));
    refreshDetails();
}

void MainWindow::applyConfiguration() {
    auto* device = selectedDevice();
    if (!device) return;
    if (modelEdit_->text().trimmed().isEmpty() && device->kind == DeviceKind::Ret) {
        QMessageBox::warning(this, tr("Dados inválidos"), tr("Informe o modelo da antena."));
        return;
    }
    if (!dateEdit_->text().isEmpty() && dateEdit_->text().size() != 6) {
        QMessageBox::warning(this, tr("Dados inválidos"), tr("A data deve usar seis dígitos no formato MMDDYY."));
        return;
    }
    if (QMessageBox::question(this, tr("Aplicar configuração"),
        tr("Gravar os dados de instalação no dispositivo %1?").arg(text(device->serial))) != QMessageBox::Yes) return;
    device->antennaModel = modelEdit_->text().trimmed().toStdString();
    device->serial = antennaSerialEdit_->text().trimmed().toStdString();
    device->baseStation = stationEdit_->text().trimmed().toStdString();
    device->sector = sectorEdit_->text().trimmed().toStdString();
    device->installationDate = dateEdit_->text().toStdString();
    device->installerId = installerEdit_->text().toStdString();
    device->technology = technologyEdit_->text().toStdString();
    device->location = locationEdit_->text().toStdString();
    device->mechanicalTilt = mechanicalTiltSpin_->value();
    device->bearing = bearingSpin_->value();
    device->height = heightSpin_->value();
    showResult(backend_->saveConfiguration(*device), tr("Configuração"));
    refreshTable();
    refreshDetails();
}

void MainWindow::showManualAddressing() {
    if (!backend_->simulator()) {
        QMessageBox::warning(this, tr("Endereçamento manual"),
            tr("A descoberta serial já atribui endereços automaticamente pela árvore XID AISG 3. O editor manual não é usado no perfil real; nenhum quadro foi enviado."));
        return;
    }
    QDialog dialog(this);
    dialog.setWindowTitle(tr("Endereçamento manual"));
    auto* layout = new QFormLayout(&dialog);
    auto* vendor = new QLineEdit(QStringLiteral("TY"));
    vendor->setMaxLength(2);
    auto* serial = new QLineEdit;
    serial->setMaxLength(17);
    auto* address = new QSpinBox;
    address->setRange(1, 254);
    address->setValue(1);
    auto* type = new QComboBox;
    type->addItems({QStringLiteral("RET"), QStringLiteral("TMA")});
    layout->addRow(tr("Vendor code"), vendor);
    layout->addRow(tr("Serial / UID"), serial);
    layout->addRow(tr("Novo endereço"), address);
    layout->addRow(tr("Tipo"), type);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    layout->addRow(buttons);
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    if (dialog.exec() != QDialog::Accepted) return;
    if (serial->text().trimmed().isEmpty()) {
        QMessageBox::warning(this, tr("Endereçamento"), tr("Informe um serial ou UID."));
        return;
    }
    if (std::any_of(devices_.begin(), devices_.end(), [address](const auto& d) { return d.address == address->value(); })) {
        QMessageBox::warning(this, tr("Endereçamento"), tr("O endereço selecionado já está em uso."));
        return;
    }
    DeviceRecord device;
    device.address = static_cast<std::uint8_t>(address->value());
    device.kind = type->currentIndex() == 0 ? DeviceKind::Ret : DeviceKind::Tma;
    device.state = DeviceState::Discovered;
    device.uid = serial->text().toStdString();
    device.serial = serial->text().toStdString();
    device.vendor = vendor->text().toStdString();
    device.product = QStringLiteral("Adicionado manualmente").toStdString();
    devices_.push_back(std::move(device));
    appendLog(tr("Dispositivo adicionado manualmente no endereço 0x%1").arg(address->value(), 2, 16, QLatin1Char('0')));
    refreshTable();
    refreshSummary();
}

void MainWindow::showAntennaCatalog() {
    QMessageBox::information(this, tr("Catálogo de antenas"),
        tr("O catálogo clean-room aceita importação futura de definições próprias. Arquivos e bancos de fabricantes do aplicativo legado não foram copiados."));
}

void MainWindow::showSettingsDialog() {
    QDialog dialog(this);
    dialog.setWindowTitle(tr("Preferências"));
    auto* layout = new QFormLayout(&dialog);
    auto* timeout = new QSpinBox;
    timeout->setRange(10, 10000);
    timeout->setValue(500);
    timeout->setSuffix(QStringLiteral(" ms"));
    auto* maximumLog = new QSpinBox;
    maximumLog->setRange(100, 50000);
    maximumLog->setValue(5000);
    auto* rawFrames = new QCheckBox(tr("Registrar quadros TX/RX"));
    rawFrames->setChecked(true);
    layout->addRow(tr("Timeout por resposta"), timeout);
    layout->addRow(tr("Linhas máximas de log"), maximumLog);
    layout->addRow(rawFrames);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Save | QDialogButtonBox::Cancel);
    layout->addRow(buttons);
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    if (dialog.exec() == QDialog::Accepted) {
        frameLog_->setMaximumBlockCount(maximumLog->value());
        appendLog(QStringLiteral("Preferências da sessão atualizadas"));
    }
}

void MainWindow::showAbout() {
    QMessageBox::about(this, tr("Sobre"),
        tr("<h3>Antenna Tilt Controller</h3>"
           "<p>Reconstrução clean-room em C++20 e Qt 6 para Linux.</p>"
           "<p>A interface foi inspirada nos fluxos operacionais do ATC Lite. O núcleo HDLC/AISG, o transporte e o simulador são implementações novas.</p>"
           "<p><b>Perfil real:</b> AISG Base 3.0.8 / ADB 3.1.7, auditado documentalmente e testado sem hardware. Certificação e interoperabilidade física ainda exigem bancada AISG.</p>"));
}

void MainWindow::showUnsupportedFirmware() {
    QMessageBox::warning(this, tr("Firmware desabilitado"),
        tr("Atualização de firmware e factory reset permanecem bloqueados nesta versão. Eles exigem perfil de fabricante, validação de imagem, recuperação testada e bancada dedicada."));
}

void MainWindow::exportSiteReport() {
    const auto path = QFileDialog::getSaveFileName(this, tr("Exportar relatório"), QStringLiteral("atc-site-report.csv"), tr("CSV (*.csv);;Texto (*.txt)"));
    if (path.isEmpty()) return;
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::critical(this, tr("Exportação"), tr("Não foi possível gravar o arquivo."));
        return;
    }
    QTextStream out(&file);
    out << "address,type,status,uid,vendor,product,serial,base_station,sector,aisg,electrical_tilt,gain,alarms\n";
    for (const auto& d : devices_) {
        out << QStringLiteral("0x%1").arg(d.address, 2, 16, QLatin1Char('0')).toUpper() << ','
            << kindName(d.kind) << ',' << stateName(d.state) << ',' << text(d.uid) << ',' << text(d.vendor) << ','
            << text(d.product) << ',' << text(d.serial) << ',' << text(d.baseStation) << ',' << text(d.sector) << ','
            << text(d.aisgVersion) << ',' << d.electricalTilt << ',' << d.gain << ',' << d.activeAlarms << '\n';
    }
    appendLog(tr("Relatório exportado: %1").arg(path));
}

void MainWindow::exportSessionLog() {
    const auto path = QFileDialog::getSaveFileName(this, tr("Exportar log"), QStringLiteral("atc-session.log"), tr("Log (*.log);;Texto (*.txt)"));
    if (path.isEmpty()) return;
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;
    QTextStream out(&file);
    out << "# Antenna Tilt Controller - atividade\n" << eventLog_->toPlainText()
        << "\n\n# Quadros HDLC / AISG\n" << frameLog_->toPlainText() << '\n';
    appendLog(tr("Log exportado: %1").arg(path));
}

void MainWindow::appendLog(const QString& message, bool frame) {
    const auto line = QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss.zzz  ")) + message;
    (frame ? frameLog_ : eventLog_)->appendPlainText(line);
}

void MainWindow::showResult(const OperationResult& result, const QString& operation) {
    appendLog(operation + QStringLiteral(": ") + text(result.message));
    operationStatus_->setText(operation + QStringLiteral(" • ") + text(result.message));
    if (!result.ok) QMessageBox::warning(this, operation, text(result.message));
}

}  // namespace atc::gui
