#include <QApplication>
#include <QByteArray>

#include "iq_analyzer_window.hpp"

int main(int argc, char* argv[]) {
    if (std::getenv("WSL_DISTRO_NAME") != nullptr ||
        std::getenv("WSL_INTEROP")     != nullptr) {
        if (std::getenv("QT_QPA_PLATFORM") == nullptr) {
            qputenv("QT_QPA_PLATFORM", QByteArray("xcb"));
        }
    }

    QApplication app(argc, argv);
    app.setOrganizationName("multi-radio");
    app.setApplicationName("iq-analyzer");

    iq_analyzer::IqAnalyzerWindow window;
    window.show();

    return app.exec();
}
