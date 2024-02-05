#ifndef EXVECTRCORE_LISTBUFFER_H
#define EXVECTRCORE_LISTBUFFER_H

#include "stdint.h"
#include "stddef.h"
#include "math.h"

#include "list.hpp"

namespace VCTR
{

    namespace Core
    {

        /**
         * ListBuffer class that can be used as queue or stack.
         * Can also be used to sort values and calculate median, average, deviation.
         */
        template <typename T, size_t SIZE>
        class ListBuffer : public List<T>
        {
        private:
            // Array for element storage
            T listBufferArray_[SIZE];

            // Should always have number of elements.
            size_t numElements_ = 0;

            // Points to index ahead of first element
            size_t front_ = 0;
            // Points to index of last element
            size_t back_ = 0;

        public:
            ListBuffer() {}

            ~ListBuffer() {} // Remember to delete data on destruction

            size_t getFront() const
            {
                return front_;
            }

            size_t getBack() const
            {
                return back_;
            }

            /**
             * @returns number of elements in ListBuffer
             */
            size_t size() const override;

            /**
             * @returns How large the ListBuffer in total is.
             */
            size_t sizeMax() const;

            /**
             * Places a new element to the front of the ListBuffer. AKA stack push.
             *
             * @param element element to be placed into ListBuffer.
             * @param overwrite Overwrites elements at back if true. Default false.
             * @return true if placed into ListBuffer.
             */
            bool placeFront(const T &element, bool overwrite = false);

            /**
             * Places a new element to the back of the ListBuffer. AKA enqueue item.
             *
             * @param element element to be placed into ListBuffer.
             * @param overwrite Overwrites elements at front if true. Default false.
             * @return true if placed into ListBuffer.
             */
            bool placeBack(const T &element, bool overwrite = false);

            /**
             * Takes a element from the front of the ListBuffer and places it into element. AKA dequeue item.
             * peekFront() wont remove element.
             * Calling this on an empty ListBuffer will simply return false.
             *
             * @param element Variable whos data will be overwritten.
             * @return true if removed from ListBuffer.
             */
            bool takeFront(T &element);

            /**
             * Takes a element from the back of the ListBuffer and places it into element.
             * peekBack() wont remove element.
             * Calling this on an empty ListBuffer will simply return false.
             *
             * @param element Variable whos data will be overwritten.
             * @return true if removed from ListBuffer.
             */
            bool takeBack(T &element);

            /**
             * Copies a element from the front of the ListBuffer and places it into element.
             * takeFront() will remove element.
             * Calling this on an empty ListBuffer will simply return false.
             *
             * @param element Variable whos data will be overwritten.
             * @return true if removed from ListBuffer.
             */
            bool peekFront(T &element);

            /**
             * Copies a element from the back of the ListBuffer and places it into element.
             * takeBack() will remove element.
             * Calling this on an empty ListBuffer will simply return false.
             *
             * @param element Variable whos data will be overwritten.
             * @return true if removed from ListBuffer.
             */
            bool peekBack(T &element);

            /**
             * Removes the element at the front. AKA dequeue.
             * Calling this on an empty ListBuffer will do nothing.
             * @param num The number of items to remove.
             */
            void removeFront(size_t num = 1);

            /**
             * Removes the element at the back.
             * Calling this on an empty ListBuffer will do nothing.
             * @param num The number of items to remove.
             */
            void removeBack(size_t num = 1);

            /**
             * Removes element that the given pointer points to.
             * @param pointerToElement Pointer to element to remove.
             * @returns true if element was found and removed.
             */
            bool removeElement(T *pointerToElement);

            /**
             * Removes element at given index.
             * @param index Index of element to be removed.
             * @returns true if element was found and removed.
             */
            bool removeElementIndex(int32_t index);

            /**
             * Places element into given index
             * @param index Index of position to place element
             * @returns true if element was placed in ListBuffer.
             */
            // bool insertElementIndex(const T &element, size_t index);

            /**
             * @returns the sum of all elements.
             */
            T getSum() const;

            /**
             * @returns the average of all elements.
             */
            T getAverage() const;

            /**
             * Sorts the elements. This will change the array!
             */
            void sortElements();

            /**
             * Sorts elements and returns median.
             * @return median.
             */
            T getMedian() const;

            /**
             * @returns the standard deviation.
             */
            T getStandardDeviation() const;

            /**
             * @returns the standard error.
             */
            T getStandardError() const;

            /**
             * Used to access ListBuffer like an array.
             * Starts from front of ListBuffer("oldest" element or first that was placed inside).
             * @returns copy of element from index.
             */
            T &operator[](size_t index) override;

            /**
             * Used to access ListBuffer like an array.
             * Const version. Returns a const reference to element.
             * Starts from front of ListBuffer("oldest" element or first that was placed inside).
             * @returns copy of element from index.
             */
            const T &operator[](size_t index) const override;

            /**
             * Safer way to access ListBuffer like an array.
             * Indexes outside of ListBuffer will wrap around ListBuffer.
             * Starts from front of ListBuffer("oldest" element or first that was placed inside).
             * @returns copy of element from index.
             */
            T &operator()(int32_t index) override;

            /**
             * Safer way to access ListBuffer like an array.
             * Const version. Returns a const reference to element.
             * Indexes outside of ListBuffer will wrap around ListBuffer.
             * Starts from front of ListBuffer("oldest" element or first that was placed inside).
             * @returns copy of element from index.
             */
            const T &operator()(int32_t index) const override;

            /**
             * Needs to be overloaded to also copy the data to instance.
             * Not doing this will cause 2 ListBuffers to share the exact same elements.
             */
            ListBuffer operator=(const ListBuffer &toBeCopied);

            /**
             * Removes all items from ListBuffer. Not computationaly intensive.
             */
            void clear();

        private:
            void quickSort(size_t left, size_t right);

            size_t quickSortPartition(size_t left, size_t right);
        };

        template <typename T, size_t SIZE>
        T ListBuffer<T, SIZE>::getStandardError() const
        {
            if (numElements_ < 2)
                return T();
            return getStandardDeviation() / sqrt(numElements_);
        }

        template <typename T, size_t SIZE>
        T ListBuffer<T, SIZE>::getStandardDeviation() const
        {

            if (numElements_ < 2)
                return T();

            T standardDev = 0;

            T avg = getAverage();

            for (size_t i = 0; i < numElements_; i++)
            {

                T diff = (*this)[i] - avg;

                standardDev = standardDev + diff * diff;
            }

            standardDev = sqrt(standardDev / (numElements_ - 1));

            return standardDev;
        }

        template <typename T, size_t SIZE>
        T ListBuffer<T, SIZE>::getMedian() const
        {

            if (numElements_ == 0)
                return T();

            T median;

            ListBuffer<T, SIZE> ListBufferSorted;

            ListBufferSorted = *this;

            ListBufferSorted.sortElements();

            if (numElements_ % 2 == 0)
            {

                median = (ListBufferSorted[numElements_ / 2 - 1] + ListBufferSorted[numElements_ / 2]) / 2;
            }
            else
            {

                median = ListBufferSorted[numElements_ / 2];
            }

            return median;
        }

        template <typename T, size_t SIZE>
        T ListBuffer<T, SIZE>::getAverage() const
        {
            if (numElements_ == 0)
                return T();
            return getSum() / numElements_;
        }

        template <typename T, size_t SIZE>
        T ListBuffer<T, SIZE>::getSum() const
        {

            T sum_ = 0;

            for (size_t i = 0; i < numElements_; i++)
            {
                sum_ = sum_ + (*this)[i];
            }

            return sum_;
        }

        template <typename T, size_t SIZE>
        size_t ListBuffer<T, SIZE>::quickSortPartition(size_t left, size_t right)
        {

            T pivot = (*this)[right];

            size_t i = left;

            for (size_t j = left; j < right; j++)
            {

                if ((*this)[j] < pivot)
                {

                    // swap
                    T temp = (*this)[i];
                    (*this)[i] = (*this)[j];
                    (*this)[j] = temp;

                    i++;
                }
            }

            // swap
            T temp = (*this)[i];
            (*this)[i] = (*this)[right];
            (*this)[right] = temp;

            return i;
        }

        template <typename T, size_t SIZE>
        void ListBuffer<T, SIZE>::quickSort(size_t left, size_t right)
        {

            if (left < right)
            {

                size_t p = quickSortPartition(left, right);
                if (p != left)
                    quickSort(left, p - 1);
                quickSort(p + 1, right);
            }
        }

        template <typename T, size_t SIZE>
        void ListBuffer<T, SIZE>::sortElements()
        {

            // Check if nothing to sort
            if (numElements_ < 2)
                return;

            quickSort(0, numElements_ - 1);
        }

        template <typename T, size_t SIZE>
        void ListBuffer<T, SIZE>::clear()
        {
            front_ = 0;
            back_ = 0;
            numElements_ = 0;
        }

        template <typename T, size_t SIZE>
        bool ListBuffer<T, SIZE>::removeElementIndex(int32_t index)
        {

            // Make sure ListBuffer isnt empty
            if (numElements_ == 0)
                return false;

            // Special case if numberElements if 1. Algorithm below cant handle this case.
            if (numElements_ == 1)
            {
                clear();
                return true;
            }

            // We have to move all elements ahead the one to be removed, one place down.
            for (size_t j = index; j < numElements_ - 1; j++)
            {

                ((*this)[j]) = ((*this)[j + 1]);
            }

            numElements_--;

            // If we exited the loop then item was not found.
            return true;
        }

        template <typename T, size_t SIZE>
        bool ListBuffer<T, SIZE>::removeElement(T *pointerToElement)
        {

            // Make sure ListBuffer isnt empty
            if (numElements_ == 0)
                return false;

            // Find element it points to
            for (size_t i = 0; i < numElements_; i++)
            {

                // Check if this is the item
                if (&((*this)[i]) == pointerToElement)
                {

                    return removeElementIndex(i);
                }
            }

            // If we exited the loop then item was not found.
            return false;
        }

        template <typename T, size_t SIZE>
        T &ListBuffer<T, SIZE>::operator[](size_t index)
        {
            if (index >= front_)
                return listBufferArray_[SIZE + front_ - index - 1];
            else
                return listBufferArray_[front_ - index - 1];
        }

        template <typename T, size_t SIZE>
        const T &ListBuffer<T, SIZE>::operator[](size_t index) const
        {
            if (index >= front_)
                return listBufferArray_[SIZE + front_ - index - 1];
            else
                return listBufferArray_[front_ - index - 1];
        }

        template <typename T, size_t SIZE>
        T &ListBuffer<T, SIZE>::operator()(int32_t index)
        {
            if (index < 0)
                index = numElements_ + index;
            return listBufferArray_[(front_ - index - 1) % numElements_];
        }

        template <typename T, size_t SIZE>
        const T &ListBuffer<T, SIZE>::operator()(int32_t index) const
        {
            if (index < 0)
                index = numElements_ + index;
            return listBufferArray_[(front_ - index - 1) % numElements_];
        }

        template <typename T, size_t SIZE>
        ListBuffer<T, SIZE> ListBuffer<T, SIZE>::operator=(const ListBuffer &toBeCopied)
        {

            size_t sizeToBeCopied = toBeCopied.available();

            for (size_t i = 0; i < sizeToBeCopied; i++)
                this->placeFront(toBeCopied[i]);

            return *this;
        }

        template <typename T, size_t SIZE>
        bool ListBuffer<T, SIZE>::peekFront(T &element)
        {

            if (numElements_ == 0)
                return false;

            element = listBufferArray_[(front_ + 1) % SIZE];

            return true;
        }

        template <typename T, size_t SIZE>
        bool ListBuffer<T, SIZE>::peekBack(T &element)
        {

            if (numElements_ == 0)
                return false;

            element = listBufferArray_[back_];

            return true;
        }

        template <typename T, size_t SIZE>
        bool ListBuffer<T, SIZE>::takeFront(T &element)
        {

            if (numElements_ == 0)
                return false;

            if (front_ == 0)
                front_ = SIZE - 1;
            else
                front_--;

            element = listBufferArray_[front_];

            numElements_--;

            return true;
        }

        template <typename T, size_t SIZE>
        bool ListBuffer<T, SIZE>::takeBack(T &element)
        {

            if (numElements_ == 0)
                return false;

            element = listBufferArray_[back_];

            back_ = (back_ + 1) % SIZE;

            numElements_--;

            return true;
        }

        template <typename T, size_t SIZE>
        size_t ListBuffer<T, SIZE>::size() const
        {
            return numElements_;
        }

        template <typename T, size_t SIZE>
        size_t ListBuffer<T, SIZE>::sizeMax() const
        {
            return SIZE;
        }

        template <typename T, size_t SIZE>
        void ListBuffer<T, SIZE>::removeFront(size_t num)
        {

            if (numElements_ < num) // If we want to remove more than we have then remove all.
                num = numElements_;

            if (front_ < num)
                front_ = SIZE + front_ - num;
            else
                front_ -= num;

            numElements_ -= num;
        }

        template <typename T, size_t SIZE>
        void ListBuffer<T, SIZE>::removeBack(size_t num)
        {

            if (numElements_ < num)
                num = numElements_;

            if (back_ + num >= SIZE)
                back_ = back_ + num - SIZE;
            else
                back_ += num;

            numElements_ -= num;
        }

        template <typename T, size_t SIZE>
        bool ListBuffer<T, SIZE>::placeFront(const T &element, bool overwrite)
        {

            if (numElements_ == SIZE)
            {

                if (overwrite)
                    removeBack();
                else
                    return false;
            }

            listBufferArray_[front_] = element;

            front_ = (front_ + 1) % SIZE;
            numElements_++;

            return true;
        }

        template <typename T, size_t SIZE>
        bool ListBuffer<T, SIZE>::placeBack(const T &element, bool overwrite)
        {

            if (numElements_ == SIZE)
            {

                if (overwrite)
                    removeFront();
                else
                    return false;
            }

            if (back_ == 0)
                back_ = SIZE - 1;
            else
                back_--;

            listBufferArray_[back_] = element;

            numElements_++;

            return true;
        }

    }
} // namespace VCTR

#endif