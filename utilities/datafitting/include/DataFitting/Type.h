#pragma once

#include <vector>
#include <array>

namespace DataFitting
{

template<typename IndexType, unsigned int IndexSize>
class Type
{
    template<typename DataType, typename TimeType>
    friend
    class Service;

public:
    Type() : data_(IndexSize) {}

    Type(std::array<double, IndexSize> list) : data_(std::begin(list), std::end(list)) {}

    double& operator[](IndexType index)
    {
        return data_[index];
    }

    const double& get(IndexType index) const
    {
        return data_[index];
    }

protected:
    std::vector<double> data_;
};

}
