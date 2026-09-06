// Учебный «updater»: UDP-сервер с упрощённым текстовым протоколом.
// Реальный miIO на пылесосе — бинарные зашифрованные пакеты на UDP:54321; здесь только иллюстрация
// потока: команда → скачать .pkg → проверить md5 → запустить sh-скрипт установки.

#include "updater/updater_app.h"

int main(int argc, char** argv)
{
    updater::UpdaterApp app(updater::ResolveInstallScript(argc, argv));
    return app.Run();
}
