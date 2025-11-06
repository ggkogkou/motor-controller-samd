#pragma once

#include "definitions.h"
#include "etl/atomic.h"
#include "etl/string.h"
#include "etl/to_string.h"
#include "etl/type_traits.h"

class Logger {
public:
        static constexpr size_t MAX_LOG_SIZE = 1024;

        Logger() {
                SERCOM3_USART_WriteCallbackRegister(&Logger::WriteDoneThunk, reinterpret_cast<uintptr_t>(this));
                tx_done.store(true);
        }

        bool writeMessage(const etl::string<MAX_LOG_SIZE>& message) {
                if (SERCOM3_USART_WriteIsBusy())
                        return false;
                tx_done.store(false);
                SERCOM3_USART_Write(const_cast<char*>(message.c_str()), message.size());
                return true;
        }

        class Line {
        public:
                explicit Line(Logger& l) : log(l) {}
                ~Line() { flush(); }

                Line& operator<<(const char* s) {
                        append(s);
                        return *this;
                }
                Line& operator<<(char c) {
                        if (buf.size() < buf.max_size())
                                buf.push_back(c);
                        return *this;
                }
                Line& operator<<(const etl::string<MAX_LOG_SIZE>& s) {
                        append(s.c_str());
                        return *this;
                }
                template <size_t N>
                Line& operator<<(const etl::string<N>& s) {
                        append(s.c_str());
                        return *this;
                }

                template <typename T>
                typename etl::enable_if<etl::is_integral<T>::value, Line&>::type operator<<(T v) {
                        etl::string<32> tmp;
                        etl::to_string(v, tmp);
                        append(tmp.c_str());
                        return *this;
                }

                Line& operator<<(float v) {
                        append_float(static_cast<double>(v));
                        return *this;
                }
                Line& operator<<(double v) {
                        append_float(v);
                        return *this;
                }
                Line& operator<<(long double v) {
                        append_float(static_cast<double>(v));
                        return *this;
                }

        private:
                static constexpr int kPrec = 6;

                void append(const char* s) {
                        if (!s)
                                return;
                        while (*s && buf.size() < buf.max_size())
                                buf.push_back(*s++);
                }

                void append_float(double v) {
                        if (buf.size() >= buf.max_size())
                                return;
                        if (v != v) {
                                append("nan");
                                return;
                        }
                        if (v > 1e300) {
                                append("inf");
                                return;
                        }
                        if (v < -1e300) {
                                append("-inf");
                                return;
                        }

                        if (v < 0) {
                                buf.push_back('-');
                                v = -v;
                                if (buf.size() >= buf.max_size())
                                        return;
                        }

                        long long ip = static_cast<long long>(v);
                        double frac = v - static_cast<double>(ip);

                        etl::string<32> tmpi;
                        etl::to_string(ip, tmpi);
                        append(tmpi.c_str());

                        if (kPrec <= 0)
                                return;
                        if (buf.size() >= buf.max_size())
                                return;

                        buf.push_back('.');
                        static const int pow10[] = {1, 10, 100, 1000, 10000, 100000, 1000000};
                        int scale = pow10[kPrec];
                        long long f = static_cast<long long>(frac * scale + 0.5);
                        if (f >= scale) {
                                ++ip;
                                f = 0;
                        }

                        etl::string<32> tmpf;
                        etl::to_string(f, tmpf);

                        for (int i = kPrec - tmpf.size(); i > 0 && buf.size() < buf.max_size(); --i)
                                buf.push_back('0');
                        append(tmpf.c_str());
                }

                void flush() {
                        if (buf.empty())
                                return;
                        if (SERCOM3_USART_WriteIsBusy()) {
                                buf.clear();
                                return;
                        }
                        log.tx_buffer_.assign(buf.begin(), buf.end());
                        buf.clear();
                        if (log.tx_buffer_.size() + 2 > log.tx_buffer_.max_size())
                                log.tx_buffer_.resize(log.tx_buffer_.max_size() - 2);
                        log.tx_buffer_ += "\r\n";
                        log.tx_done.store(false);
                        SERCOM3_USART_Write(const_cast<char*>(log.tx_buffer_.c_str()), log.tx_buffer_.size());
                }

                Logger& log;
                etl::string<MAX_LOG_SIZE> buf{};
        };

        Line operator<<(const char* s) {
                Line p(*this);
                p << s;
                return p;
        }
        Line operator<<(char c) {
                Line p(*this);
                p << c;
                return p;
        }
        Line operator<<(const etl::string<MAX_LOG_SIZE>& s) {
                Line p(*this);
                p << s;
                return p;
        }
        template <size_t N>
        Line operator<<(const etl::string<N>& s) {
                Line p(*this);
                p << s;
                return p;
        }
        template <typename T>
        typename etl::enable_if<etl::is_integral<T>::value, Line>::type operator<<(T v) {
                Line p(*this);
                p << v;
                return p;
        }
        Line operator<<(float v) {
                Line p(*this);
                p << v;
                return p;
        }
        Line operator<<(double v) {
                Line p(*this);
                p << v;
                return p;
        }
        Line operator<<(long double v) {
                Line p(*this);
                p << v;
                return p;
        }

        bool isReady() const { return !SERCOM3_USART_WriteIsBusy(); }
        bool txComplete() const { return tx_done.load(); }

private:
        static void WriteDoneThunk(uintptr_t context) {
                auto* self = reinterpret_cast<Logger*>(context);
                if (self)
                        self->onWriteDone();
        }

        void onWriteDone() { tx_done.store(true); }

        etl::string<MAX_LOG_SIZE + 2> tx_buffer_{};
        etl::atomic<bool> tx_done{true};
};
