

#include "SerialPort.h"




SerialPort::SerialPort(void) : end_of_line_char_('\n')
{
  mFirstReception = 0;
}

SerialPort::~SerialPort(void)
{
	stop();
}

char SerialPort::end_of_line_char() const
{
	return this->end_of_line_char_;
}

void SerialPort::end_of_line_char(const char &c)
{
	this->end_of_line_char_ = c;
}

bool SerialPort::start(const char *com_port_name, int baud_rate)
{
	boost::system::error_code ec;

	if (port_) {
		std::cout << "error : port is already opened..." << std::endl;
		return false;
	}

	port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
	port_->open(com_port_name, ec);
	if (ec) {
		std::cout << "error : port_->open() failed...com_port_name="
			<< com_port_name << ", e=" << ec.message().c_str() << std::endl;
		return false;
	}

	// option settings...
	port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
	port_->set_option(boost::asio::serial_port_base::character_size(7));
	port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::two));
	port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
	port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::type::hardware));

	boost::thread t(boost::bind(&boost::asio::io_service::run, &io_service_));

	async_read_some_();

	return true;
}

void SerialPort::stop()
{
	boost::mutex::scoped_lock look(mutex_);

	if (port_) {
		port_->cancel();
		port_->close();
		port_.reset();
	}
	io_service_.stop();
	io_service_.reset();
}

int SerialPort::write_some(const std::string &buf)
{
	return write_some(buf.c_str(), buf.size());
}

int SerialPort::write_some(const char *buf, const int &size)
{
	boost::system::error_code ec;

	if (!port_) return -1;
	if (size == 0) return 0;

	return port_->write_some(boost::asio::buffer(buf, size), ec);
}

void SerialPort::async_read_some_()
{
	if (port_.get() == NULL || !port_->is_open()) return;

	port_->async_read_some(
		boost::asio::buffer(read_buf_raw_, SERIAL_PORT_READ_BUF_SIZE),
		boost::bind(
			&SerialPort::on_receive_,
			this, boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred));
}

void SerialPort::on_receive_(const boost::system::error_code& ec, size_t bytes_transferred)
{
	boost::mutex::scoped_lock look(mutex_);

	if (port_.get() == NULL || !port_->is_open())
	{
		return;
	}
	if (ec) 
  {
		async_read_some_();
		return;
	}

	for (unsigned int i = 0; i < bytes_transferred; ++i) {
		char c = read_buf_raw_[i];
		if (c == end_of_line_char_) {
			this->on_receive_(read_buf_str_);
			read_buf_str_.clear();
		}
		else {
			read_buf_str_ += c;
		}
	}

	async_read_some_();
}

void SerialPort::on_receive_(const std::string &data)
{
  boost::chrono::duration<double> wSec;
  int wDataStart = 0;
  int wUnitStart = 0;
  
  wDataStart = data.find_first_of(' ');
  if (wDataStart != std::string::npos && wDataStart + 1 < data.size() )
  {
    wDataStart++;
  }

  wUnitStart = data.find_first_of(' ', wDataStart);
  if (wUnitStart != std::string::npos && wUnitStart + 1 < data.size())
  {
    wUnitStart++;
  }
  else
  {
    wUnitStart = -1;
  }


  if (mFirstReception == 0)
  {
    mFirstReception = 1;
    mTimer = boost::chrono::system_clock::now();
    wSec = mTimer - mTimer;
    std::cout << "Time(Second) ; " << data.substr(0, wDataStart - 1);
    if (wUnitStart != -1)
    {
      std::cout << "(" << data.substr(wUnitStart, data.size() - wUnitStart) << ")";
    }
    std::cout << std::endl;
    if (mOuputFile.is_open())
    {
      mOuputFile << "Time(Second) ; " << data.substr(0, wDataStart - 1) ;
      if (wUnitStart != -1)
      {
        mOuputFile << "(" << data.substr(wUnitStart, data.size() - wUnitStart) << ")";
      }
      mOuputFile << std::endl;
    }
  }
  else
  {
    wSec = boost::chrono::system_clock::now() - mTimer;
  }
  
	std::cout << wSec.count() << "; " << data.substr(wDataStart, wUnitStart - wDataStart) << std::endl;
  if (mOuputFile.is_open())
  {
    mOuputFile << wSec.count() << "; " << data.substr(wDataStart, wUnitStart - wDataStart) << std::endl;
  }
  char wEnter[2] = { 27,0 };
  this->write_some(wEnter, 1);
}

void SerialPort::SetDestinationFile(const char* iFilename)
{
  if (mOuputFile.is_open() == true)
  {
    mOuputFile.close();
  }
  mOuputFile.open(iFilename, std::ofstream::out);
}