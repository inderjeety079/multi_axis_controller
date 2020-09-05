#ifndef MESSAGE_QUEUE_H
#define MESSAGE_QUEUE_H

#include <condition_variable>
#include <list>
#include <mutex>
#include <queue>

template <typename T> class MessageQueue {
private:
  std::queue<T> queue;
  std::mutex mutexLock;
  std::condition_variable dataCondition;

public:
  /*!
   * \brief empty
   * Check if queue is empty
   * \return
   * True if queue is empty else false
   */
  typedef std::shared_ptr<MessageQueue<T>> MessageQueueSPtrType;
  typedef std::shared_ptr<MessageQueue<std::string>> MessageQueueStdStringSPtrType;
  bool dataConditionBreak;
  bool empty() {
    std::unique_lock<std::mutex> lock(mutexLock);
    return queue.empty();
  }

  /*!
   * \brief push
   * Pushes the value in the queue
   * \param value
   * value to be pushed in the queue with type T
   */
  void push(T value) {
    std::unique_lock<std::mutex> lock(mutexLock);
    queue.push(value);
    dataCondition.notify_one();
  }

  /*!
   * \brief pop
   * Do not use this function. Will take lot of cpu
   * cycles when called in while 1
   * \return
   */
  T pop() {
    std::unique_lock<std::mutex> lock(mutexLock);
    T value = queue.front();
    queue.pop();
    return value;
  }

  /*!
   * \brief waitAndPop
   * \param value
   */
  void waitAndPop(T &value) {
    std::unique_lock<std::mutex> lock(mutexLock);
    dataCondition.wait(lock, [this] { return (!queue.empty()); });
  }

  /*!
 * \brief waitAndPop
 * \param value
 */
  void wait() {
      std::unique_lock<std::mutex> lock(mutexLock);
      dataCondition.wait(lock, [this] { return (!queue.empty()); });
  }

  void waitAndPop(T &value,std::unique_lock<std::mutex> user_lock) {
    std::unique_lock<std::mutex> lock(user_lock);
    dataCondition.wait(lock, [this] { return ((!queue.empty()) || dataConditionBreak); });
  }

  void emptyQueue() {
    std::unique_lock<std::mutex> lock(mutexLock);
    while (!queue.empty()) {
      break;
    }
  }

  /**
   * \brief breaks the wait of condition variable
   * 
   */
  void breakWait() {
    dataConditionBreak = true;
    dataCondition.notify_one();
  }

  void resetBreakWait() {
    dataConditionBreak = false;
  }
};

#endif // MESSAGE_QUEUE
