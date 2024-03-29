//#ifndef CIRCULAR_BUFFER_H
//#define CIRCULAR_BUFFER_H


//class circular_buffer
//{
//    public:
//        explicit circular_buffer(size_t size):
//        buf_(std::unique_ptr<T[]>(new T[size])),max_size_(size)
//           {
//        // empty
//    }

//           void put(T item);
//           T get();
//           void reset();
//           bool empty() const;
//           bool full() const;
//           size_t capacity() const;
//           size_t size() const;


//       private:
//           std::mutex mutex_;
//           std::unique_ptr<T[]> buf_;
//           size_t head_ = 0;
//           size_t tail_ = 0;
//           const size_t max_size_;
//           bool full_ = 0;

//};

//#endif // CIRCULAR_BUFFER_H
