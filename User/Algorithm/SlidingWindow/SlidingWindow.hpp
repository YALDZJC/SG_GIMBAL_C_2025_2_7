
#pragma once

#include <stdint.h> // uint8_t

namespace Alg::SW
{

/**
 * @brief 滑动窗口，用于检测数据是否超过阈值
 * 
 * @tparam Type     数据类型
 * @tparam Max_Size 最大窗口大小
 */
template <typename Type, uint32_t Max_Size = 200> class SlidingWindowDetector
{
  private:
    Type data[Max_Size]; // 数据数组
    uint8_t capacity;    // 实际使用的队列容量
    uint8_t head;        // 队首索引
    uint8_t tail;        // 队尾索引
    uint8_t size;        // 当前队列大小

    Type sum;       // 当前窗口累计值
    Type threshold; // 检测阈值

  public:
    /**
     * @brief 构造函数
     * @param windowSize 窗口大小
     * @param detectionThreshold 检测阈值
     */
    SlidingWindowDetector(uint32_t windowSize, Type detectionThreshold)
    {
        // 确保窗口大小不超过最大限制
        capacity = (windowSize < Max_Size) ? windowSize : Max_Size;
        head = 0;
        tail = 0;
        size = 0;
        sum = 0.0f;
        threshold = detectionThreshold;

        // 初始化数组
        for (int i = 0; i < Max_Size; i++)
        {
            data[i] = 0.0f;
        }
    }

    /**
     * @brief 添加新值并检测是否触发阈值
     * @param value 新的数据值
     * @return 是否触发阈值
     */
    bool addValue(float value)
    {
        // 添加新值到队列
        data[tail] = value;
        tail = (tail + 1) % capacity;
        sum += value;

        // 增加队列大小，如果已满则移除最早的值
        if (size < capacity)
        {
            size++;
        }
        else
        {
            sum -= data[head];
            head = (head + 1) % capacity;
        }

        // 检测是否超过阈值
        if (sum > threshold)
        {
            return true;
        }

        return false;
    }

    /**
     * @brief 重置滑动窗口
     */
    void reset()
    {
        head = 0;
        tail = 0;
        size = 0;
        sum = 0.0f;
    }

    /**
     * @brief 获取当前窗口累计值
     * @return 当前窗口累计值
     */
    Type getSum() const
    {
        return sum;
    }

    /**
     * @brief 获取当前窗口大小
     * @return 当前窗口中的元素数量
     */
    int getSize() const
    {
        return size;
    }
};
} // namespace Alg::SW
