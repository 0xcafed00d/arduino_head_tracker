#ifndef LOG_H
#define LOG_H

#include "Arduino.h"

class Logger {
   public:
	class LogProxy {
	   public:
		LogProxy(Logger& logger) : m_logger(logger) {
			m_logger.m_proxycount++;
		}

		~LogProxy() {
			m_logger.m_proxycount--;

			if (m_logger.m_enabled && m_logger.m_proxycount == 0) {
				m_logger.m_stream.println();
			}
		}
		LogProxy(const LogProxy& that) : m_logger(that.m_logger) {
			m_logger.m_proxycount++;
		}

		template <typename T>
		LogProxy operator<<(const T& v) {
			m_logger << v;
			return LogProxy(m_logger);
		};

	   private:
		Logger& m_logger;
	};
	friend LogProxy;

	Logger(Stream& strm) : m_stream(strm) {
	}

	Stream& stream() {
		return m_stream;
	}

	template <typename T>
	LogProxy operator<<(const T& v) {
		if (m_enabled) {
			m_stream.print(v);
			m_stream.print(' ');
		}
		return LogProxy(*this);
	};

	bool enabled() const {
		return m_enabled;
	}

	void enable(bool enable) {
		m_enabled = enable;
	}

   private:
	Stream& m_stream;
	int m_proxycount = 0;
	bool m_enabled = true;
};

extern Logger logr;

#endif /* LOG_H */
