#ifndef _JETBOT_CONTROLLER_I2C_HPP_
#define _JETBOT_CONTROLLER_I2C_HPP_

#include <memory>
#include <string>
#include <cstdint>

namespace jetbot_controller
{
  /**
   * @class I2c
   * @brief Communicates with a I2C device on linux
   */
  class I2c
  {
  public:
    typedef std::unique_ptr<I2c> Ptr;
    typedef std::unique_ptr<const I2c> ConstPtr;

    class Device
    {
    public:
      Device(const std::string &bus_path, const std::uint8_t address);

      const std::string &getBusPath() const noexcept;
      std::uint8_t getAddress() const noexcept;

    private:
      std::string bus_path_;
      std::uint8_t address_;
    };

    ~I2c();

    /**
     * @brief Open a connection to a given I2C device
     */
    static Ptr open(const Device &device);

    /**
     * @brief Writes a raw buffer to the device
     */
    std::size_t write(const std::uint8_t *const data, const std::size_t length);
    
    /**
     * @brief Writes multiple bytes to a register
     */
    std::size_t write(const std::uint8_t reg, const std::uint8_t *const data, const std::size_t length);
    
    /**
     * @brief Writes a single byte to a register
     */
    std::size_t write(const std::uint8_t reg, const std::uint8_t value);

    /**
     * @brief Reads a raw buffer from the device
     */
    std::size_t read(std::uint8_t *const data, const std::size_t length);
    
    /**
     * @brief Reads multiple bytes from a register
     */
    std::size_t read(const std::uint8_t reg, std::uint8_t *const data, const std::size_t length);
    
    /**
     * @brief Reads a single byte from a register
     */
    std::size_t read(const std::uint8_t reg);
    
    /**
     * @brief Reads a single byte, applies a mask, and writes the result back
     */
    std::size_t mask(const std::uint8_t reg, const std::uint8_t mask);

  private:
    I2c(const int fd, const std::uint8_t address);

    int fd_;
    std::uint8_t address_;
  };
}

#endif