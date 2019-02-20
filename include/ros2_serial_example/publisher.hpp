#pragma once

#include <cstdint>

class Publisher
{
public:
  virtual void dispatch(char data_buffer[], ssize_t length) = 0;
};
