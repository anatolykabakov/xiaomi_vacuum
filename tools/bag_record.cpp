// bag_record — записывает всё, что робот публикует по ZMQ (PUB tcp://robot:9091), в бег.
//
//   bag_record --host 192.168.0.137 --out drive.bag [--duration 120] [--topics slam/map,slam/pose]
//
// Формат файла (см. docs/BAGS.md): заголовок "XRBAG1\n", далее записи подряд:
//   uint64 LE  время приёма на хосте, мкс с эпохи
//   uint32 LE  длина сообщения
//   байты      xiaomi.robot.ZmqMessage (тот же protobuf, что идёт по сети)
// Остановка — Ctrl+C или --duration; в конце печатается сводка по топикам.

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>

#include <tclap/CmdLine.h>
#include <zmq.hpp>

#include "zmq_message.pb.h"

namespace
{

std::atomic<bool> g_stop{false};

void OnSignal(int)
{
    g_stop = true;
}

uint64_t NowUs()
{
    using namespace std::chrono;
    return static_cast<uint64_t>(duration_cast<microseconds>(system_clock::now().time_since_epoch()).count());
}

void PutU32(std::ostream& out, uint32_t v)
{
    const unsigned char b[4] = {
        static_cast<unsigned char>(v), static_cast<unsigned char>(v >> 8),
        static_cast<unsigned char>(v >> 16), static_cast<unsigned char>(v >> 24)};
    out.write(reinterpret_cast<const char*>(b), 4);
}

void PutU64(std::ostream& out, uint64_t v)
{
    PutU32(out, static_cast<uint32_t>(v));
    PutU32(out, static_cast<uint32_t>(v >> 32));
}

std::set<std::string> SplitTopics(const std::string& list)
{
    std::set<std::string> topics;
    std::stringstream ss(list);
    std::string item;
    while (std::getline(ss, item, ','))
        if (!item.empty()) topics.insert(item);
    return topics;
}

}  // namespace

int main(int argc, char** argv)
{
    std::string host, out_path, topics_arg;
    int port = 0;
    double duration = 0;
    try
    {
        TCLAP::CmdLine cmd("Запись ZMQ-топиков робота в бег", ' ', "1.0");
        TCLAP::ValueArg<std::string> host_arg("H", "host", "адрес робота", false, "192.168.0.137", "ip", cmd);
        TCLAP::ValueArg<int> port_arg("p", "port", "PUB-порт робота", false, 9091, "port", cmd);
        TCLAP::ValueArg<std::string> out_arg("o", "out", "файл бега", false, "robot.bag", "path", cmd);
        TCLAP::ValueArg<double> dur_arg("d", "duration", "длительность, с (0 — до Ctrl+C)", false, 0.0, "sec", cmd);
        TCLAP::ValueArg<std::string> topics_arg_("t", "topics", "какие топики писать, через запятую (пусто — все)", false, "", "list", cmd);
        cmd.parse(argc, argv);
        host = host_arg.getValue();
        port = port_arg.getValue();
        out_path = out_arg.getValue();
        duration = dur_arg.getValue();
        topics_arg = topics_arg_.getValue();
    }
    catch (const TCLAP::ArgException& e)
    {
        std::cerr << e.error() << " (" << e.argId() << ")\n";
        return 2;
    }
    const std::set<std::string> wanted = SplitTopics(topics_arg);

    std::ofstream out(out_path, std::ios::binary);
    if (!out)
    {
        std::cerr << "не открыть " << out_path << "\n";
        return 1;
    }
    out.write("XRBAG1\n", 7);

    zmq::context_t ctx(1);
    zmq::socket_t sub(ctx, zmq::socket_type::sub);
    sub.set(zmq::sockopt::rcvtimeo, 200);
    sub.set(zmq::sockopt::subscribe, "");
    const std::string endpoint = "tcp://" + host + ":" + std::to_string(port);
    sub.connect(endpoint);
    std::cerr << "bag_record: " << endpoint << " -> " << out_path << (duration > 0 ? " (" + std::to_string(duration) + " с)" : " (Ctrl+C)") << "\n";

    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    std::map<std::string, uint64_t> count, bytes;
    uint64_t total = 0, dropped = 0;
    const uint64_t t0 = NowUs();
    uint64_t first_ts = 0, last_ts = 0;
    while (!g_stop)
    {
        if (duration > 0 && NowUs() - t0 > static_cast<uint64_t>(duration * 1e6)) break;
        zmq::message_t msg;
        if (!sub.recv(msg, zmq::recv_flags::none)) continue;

        xiaomi::robot::ZmqMessage parsed;
        if (!parsed.ParseFromArray(msg.data(), static_cast<int>(msg.size())))
        {
            ++dropped;
            continue;
        }
        if (!wanted.empty() && !wanted.count(parsed.topic())) continue;

        const uint64_t now = NowUs();
        PutU64(out, now);
        PutU32(out, static_cast<uint32_t>(msg.size()));
        out.write(static_cast<const char*>(msg.data()), static_cast<std::streamsize>(msg.size()));
        ++count[parsed.topic()];
        bytes[parsed.topic()] += msg.size();
        ++total;
        if (first_ts == 0) first_ts = now;
        last_ts = now;
    }
    out.close();

    std::cerr << "записано " << total << " сообщений, " << (last_ts > first_ts ? (last_ts - first_ts) / 1e6 : 0.0) << " с";
    if (dropped) std::cerr << ", не распарсилось " << dropped;
    std::cerr << "\n";
    for (const auto& [topic, n] : count)
        std::cerr << "  " << topic << ": " << n << " (" << bytes[topic] / 1024 << " КиБ)\n";
    return 0;
}
