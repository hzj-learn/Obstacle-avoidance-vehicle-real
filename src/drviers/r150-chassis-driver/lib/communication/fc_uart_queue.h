/*
 * @Description: 
 * @Version: 0.0
 * @Autor: LQF
 * @Date: 2020-09-15 11:51:43
 * @LastEditors: LQF
 * @LastEditTime: 2020-09-15 14:49:30
 */
#pragma once

#include <mutex>
#include <queue>

namespace xag_chassis
{
namespace communication
{

class FCUartQueue
{
  public:
    FCUartQueue() {}
    FCUartQueue& operator=(const FCUartQueue& other) = delete;
    FCUartQueue(const FCUartQueue& other) = delete;

    ~FCUartQueue() {}

    void Enqueue(const uint8_t* element, const size_t num)
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        for(size_t _i = 0; _i < num; ++_i)
        {
            queue_.push(element[_i]);
        }
    }

    size_t Dequeue(uint8_t* const element, const size_t num)
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        size_t _dequeue_size = 0;
        if(queue_.empty())
        {
            return _dequeue_size;
        }

        for(size_t _i = 0; _i < num; ++_i)
        {
            if(queue_.empty())
            {
                return _dequeue_size;
            }
            element[_i] = std::move(queue_.front());
            queue_.pop();
            ++_dequeue_size;
        }

        return _dequeue_size;
    }

    size_t Size()
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        return queue_.size();
    }

    bool Empty()
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        return queue_.empty();
    }

  private:
    std::mutex queue_mutex_;
    std::queue<uint8_t> queue_;  
};

}
}