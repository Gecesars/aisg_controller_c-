#include "main_window.hpp"

#include <QApplication>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QImage>
#include <QIcon>
#include <QPainter>
#include <QStyleFactory>
#include <QTimer>

int main(int argc, char* argv[]) {
    QApplication application(argc, argv);
    QApplication::setOrganizationName(QStringLiteral("ATC Open Controller"));
    QApplication::setOrganizationDomain(QStringLiteral("local.atc"));
    QApplication::setApplicationName(QStringLiteral("Antenna Tilt Controller"));
    QApplication::setApplicationVersion(QStringLiteral("0.1.0"));
    QApplication::setStyle(QStyleFactory::create(QStringLiteral("Fusion")));
    QApplication::setWindowIcon(QIcon::fromTheme(QStringLiteral("antenna-tilt-controller")));

    QCommandLineParser parser;
    parser.setApplicationDescription(QStringLiteral("Controlador AISG/RET clean-room para Linux"));
    parser.addHelpOption();
    parser.addVersionOption();
    QCommandLineOption smokeOption(QStringLiteral("smoke-test"),
        QStringLiteral("Abre a interface offscreen e encerra automaticamente."));
    QCommandLineOption demoOption(QStringLiteral("demo-test"),
        QStringLiteral("Executa conexão e descoberta no simulador e encerra."));
    QCommandLineOption screenshotOption(QStringLiteral("screenshot"),
        QStringLiteral("Salva uma captura PNG do cenário simulado."),
        QStringLiteral("arquivo"));
    parser.addOption(smokeOption);
    parser.addOption(demoOption);
    parser.addOption(screenshotOption);
    parser.process(application);

    atc::gui::MainWindow window(atc::gui::makeControllerBackend());
    window.show();

    if (parser.isSet(demoOption) || parser.isSet(screenshotOption)) {
        QTimer::singleShot(0, &window, [&window] { window.runDemoScenario(); });
        if (parser.isSet(screenshotOption)) {
            const auto path = parser.value(screenshotOption);
            QTimer::singleShot(2300, &window, [&window, path] {
                QImage image(window.size() * window.devicePixelRatioF(), QImage::Format_ARGB32_Premultiplied);
                image.setDevicePixelRatio(window.devicePixelRatioF());
                image.fill(Qt::transparent);
                QPainter painter(&image);
                window.render(&painter);
                image.save(path, "PNG");
            });
        }
        QTimer::singleShot(2700, &application, &QCoreApplication::quit);
    } else if (parser.isSet(smokeOption)) {
        QTimer::singleShot(750, &application, &QCoreApplication::quit);
    }
    return application.exec();
}
