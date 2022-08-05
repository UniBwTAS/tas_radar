#pragma once

/*
 * DataFitting Service
 * -------------------
 * replaces and enhances PoseService
*/

#include <vector>
#include "Type.h"

namespace DataFitting
{

// Configuration
const int DefaultBufferSize = 2048;        // default/initial buffer size
const int MaximumBufferSize = 1048576;    // maximum buffer size, must not exceed maximum range of variable - 1

// Enumeration for mode (quality indicator) of pose estimation.
enum class ComputationStatus
{
    interpolation,  // the requested point of time is in between of two stored samples. Estimation
    // using interpolation
            extrapolation,  // the requested point of time is outside of the samples in the buffer. Rough
    // estimation using extrapolation, use with care
            approximated,   // only one sample has been acquired. This one has been returned - no
    // computational estimation possible
            none,           // no samples have been acquired. Estimation not possible, result undefined
};

enum class ExtrapolationMode
{
    none,           // use the nearest sample in time domain
    linear,         // use linear extrapolation using two samples
};

template<typename DataType, typename TimeType = double>
class Service
{
    static_assert(std::is_base_of<DataType, DataType>::value,
                  "DataFitting::Service only supports DataFitting::Type-derived samples.");

public:
    struct Sample
    {
        TimeType time;
        DataType data;
    };

public:
    Service();

    // Set the configuration for the internal buffer. Flushes buffer. Returns true on success
    bool setConfiguration(unsigned int sizeBuffer);

    // Acquires and stores a time and pose tuple. This tuple has to be pushed externally. Returns true on success
    bool registerSample(const Sample& sample);

    // Acquires and stores a time and pose tuple. This tuple has to be pushed externally. Returns true on success
    bool registerValue(const DataType& value, TimeType timestamp);

    // Get internal buffer size
    unsigned int getBufferSize() const;

    // Get internal buffer fill level
    unsigned int getBufferFilllevel() const;

    // Check buffer for emptiness
    bool isEmpty() const;

    // Check buffer for fullness
    bool isFull() const;

    // Get entity at buffer's head
    const Sample& getNewestSample(bool* ok = NULL) const;

    // Get entity at buffer's tail
    const Sample& getOldestSample(bool* ok = NULL) const;

    // Get entity at index n from buffer's head (index n = 0 -> newest pose, index n = getBufferFilllevel() - 1 -> oldest pose)
    const Sample& getSampleAtIndex(unsigned int n, bool* ok = NULL) const;

    // Flush buffer
    void flush();

    // Get an estimation for the pose at given timestamp
    const DataType getValueAtTime(const TimeType timestamp, ComputationStatus* status = NULL, ExtrapolationMode extrapolation = ExtrapolationMode::none) const;

private:
    // Buffer pointer
    std::vector<Sample> buffer_;

    // Empty default entity
    const Sample empty_sample_{};

    // Buffer head index. Interpretation: the next FREE slot
    unsigned int head_{0};

    // Buffer length. Contains number of occupied slots
    unsigned int length_{0};

    // Buffer (maximum) size
    unsigned int full_size_{DefaultBufferSize};

    // Inter-/Extrapolation computation
    DataType computeEstimation(const Sample& A, const Sample& B, const TimeType timestamp) const;
};

template<typename DataType, typename TimeType>
Service<DataType, TimeType>::Service()
{
    setConfiguration(DefaultBufferSize);
    flush();
}

template<typename DataType, typename TimeType>
bool Service<DataType, TimeType>::setConfiguration(unsigned int sizeBuffer)
{
    if(sizeBuffer > MaximumBufferSize)
        return false;

    full_size_ = sizeBuffer;

    buffer_.clear();
    //buffer_.reserve(full_size_);
    //std::fill_n(buffer_.begin(), full_size_, empty_sample_);
    buffer_.resize(full_size_);

    return true;
}

template<typename DataType, typename TimeType>
bool Service<DataType, TimeType>::registerSample(const Service::Sample& sample)
{
    // TODO: time check. Timestamp should be higher than current latest pose. If not, sort it in? Or neglect it? Mind ADTF can run backwards...
    if((length_ != 0) && (buffer_[(head_ - 1)%full_size_].time >
                          sample.time))    // 2nd evaluation only processed on true when 1st condition is true
    {
        // the yet obtained pose timestamp is OLDER than the newest stored one. Possible reasons:
        //		- ADTF might seek or run backwards
        //		- wrong order already in recoder dump due to bus transmit indeterminism (eg. CAN multiple/parallel hardware transmit queues combined with identifier change during bus full load)
        // in second case it might be the best choice to sort the pose in (only in case the sample is not older than the oldest obtained sample, otherwise discard it)
        // but the safest way is to flush the buffer probably...
        flush();

        // throw warning to user's console...
        // ... here. (TODO)
    }

    // we always register a new pose, regardless if next slot is free or occupied (in this application, slots are not poped out anyway)
    buffer_[head_] = sample;

    // set head index to the next "free" slot ("free" in terms of next to-be-occupied slot)
    head_ = (head_ + 1)%full_size_;

    // if buffer is not fully populated, update valid buffer length remarker
    if(length_ < full_size_)
        length_ += 1;

    return true;
}

template<typename DataType, typename TimeType>
bool Service<DataType, TimeType>::registerValue(const DataType& value, TimeType timestamp)
{
    return registerSample(Service::Sample{.time = timestamp, .data = value});
}

template<typename DataType, typename TimeType>
unsigned int Service<DataType, TimeType>::getBufferSize() const
{
    return full_size_;
}

template<typename DataType, typename TimeType>
unsigned int Service<DataType, TimeType>::getBufferFilllevel() const
{
    return length_;
}

template<typename DataType, typename TimeType>
bool Service<DataType, TimeType>::isEmpty() const
{
    return length_ == 0;
}

template<typename DataType, typename TimeType>
bool Service<DataType, TimeType>::isFull() const
{
    return length_ == full_size_;
}

template<typename DataType, typename TimeType>
const typename Service<DataType, TimeType>::Sample& Service<DataType, TimeType>::getNewestSample(bool* ok) const
{
    const Sample* result;
    bool success;

    if(length_ > 0)
    {
        result = &buffer_[(head_ - 1)%full_size_];
        success = true;
    }
    else
        success = false;

    if(ok != NULL)
        *ok = success;

    return *result;
}

template<typename DataType, typename TimeType>
const typename Service<DataType, TimeType>::Sample& Service<DataType, TimeType>::getOldestSample(bool* ok) const
{
    const Sample* result;
    bool success;

    if(length_ > 0)
    {
        result = &buffer_[(head_ - length_)%full_size_];
        success = true;
    }
    else
        success = false;

    if(ok != NULL)
        *ok = success;

    return *result;
}

template<typename DataType, typename TimeType>
const typename Service<DataType, TimeType>::Sample&
Service<DataType, TimeType>::getSampleAtIndex(unsigned int n, bool* ok) const
{
    const Sample* result;
    bool success;

    if(length_ >= (1 + n))
    {
        result = &buffer_[(head_ - (1 + n))%full_size_];
        success = true;
    }
    else
        success = false;

    if(ok != NULL)
        *ok = success;

    return *result;
}

template<typename DataType, typename TimeType>
void Service<DataType, TimeType>::flush()
{
    length_ = 0;
    head_ = 0;
}

template<typename DataType, typename TimeType>
const DataType Service<DataType, TimeType>::getValueAtTime(const TimeType timestamp, ComputationStatus* status,
                                                           ExtrapolationMode extrapolation) const
{
    DataType data;
    ComputationStatus estimation;

    // Check: do we have enough poses in buffer? (linear interpolation: 2 samples necessary)
    if(length_ == 0)
    {
        estimation = ComputationStatus::none;
    }
    else if(length_ == 1)
    {
        estimation = ComputationStatus::approximated;
        data = buffer_[(head_ - 1)%full_size_].data;
    }
    else // we have at least two samples. Check if the desired sample is inside of range...
    {
        // check if requested timestamp is older than our oldest pose timestamp
        if(getOldestSample().time >= timestamp)
        {
            estimation = ComputationStatus::extrapolation;
            if(extrapolation == ExtrapolationMode::linear)
                data = computeEstimation(getSampleAtIndex(length_ - 1), getSampleAtIndex(length_ - 2), timestamp);
            else
                data = getOldestSample().data;

        }
        else if(getNewestSample().time <= timestamp)
        {
            estimation = ComputationStatus::extrapolation;
            if(extrapolation == ExtrapolationMode::linear)
                data = computeEstimation(getSampleAtIndex(0), getSampleAtIndex(1), timestamp);
            else
                data = getNewestSample().data;
        }
        else
        {
            // "actual wanted" interpolation
            estimation = ComputationStatus::interpolation;

            // search for index (we know at this point that the desired index is _in_ the range)
            unsigned int index = 1;
            bool found = false;

            while(index <= (length_ - 1))    // unnecessary safety condition
            {
                if(getSampleAtIndex(index).time <= timestamp)
                {
                    // we found the latest "oldest" pose sample
                    found = true;
                    break;
                }

                index += 1;
            }

            if(found)
            {
                data = computeEstimation(getSampleAtIndex(index - 1), getSampleAtIndex(index), timestamp);
            }
            else
            {
                // assert message... must never happen
                estimation = ComputationStatus::none;
            }
        }
    }

    if(status)
        *status = estimation;

    return data;
}

template<typename DataType, typename TimeType>
DataType Service<DataType, TimeType>::computeEstimation(const Service::Sample& A, const Service::Sample& B,
                                                        const TimeType timestamp) const
{
    DataType data;

    static_assert(!std::is_unsigned<TimeType>::value, "TimeType has to be signed for sad reasons");

    // hint: if any timestamp difference = 0, the division through zero will make the result +/- inf. There is not yet any better fallback concept

    for(unsigned int i = 0; i < data.data_.size(); i += 1)
        data.data_[i] =
                A.data.data_.at(i) + (timestamp - A.time)*(A.data.data_.at(i) - B.data.data_.at(i))/(A.time - B.time);

//    pose.x =	poseA.data.x   + ((long long signed) timestamp - (long long signed) poseA.time) * (poseA.data.x   - poseB.data.x  ) / ((long long signed) poseA.time - (long long signed) poseB.time)  ;
//    pose.y =	poseA.data.y   + ((long long signed) timestamp - (long long signed) poseA.time) * (poseA.data.y   - poseB.data.y  ) / ((long long signed) poseA.time - (long long signed) poseB.time)  ;
//    pose.yaw = poseA.data.yaw + ((long long signed) timestamp - (long long signed) poseA.time) * (poseA.data.yaw - poseB.data.yaw) / ((long long signed) poseA.time - (long long signed) poseB.time) ;

    return data;
}
}
