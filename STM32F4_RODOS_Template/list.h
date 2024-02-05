#ifndef EXVECTRCORE_LIST_H
#define EXVECTRCORE_LIST_H

#include "stddef.h"
#include "stdint.h"

namespace VCTR
{

    namespace Core
    {   

        template <typename TYPE>
        class EmptyList;

        /**
         * This is a abstract class for defining the interface for different list data types. Some examples are list array, list fixed.
         * Each different type has its pros and cons and different behaviors.
         * @param TYPE type of data to store in List
         */
        template <typename TYPE>
        class List
        {
        private:

        public:
            virtual ~List() {} // Virtual destructor. As needed due to expected inheretance of this class.

            /**
             * @brief Used for returning an empty list.
             * @return an empty list reference object.
             */
            static const List<TYPE> &empty(); 

            /**
             * @brief Will return the size of the list. (Aka the length)
             */
            virtual size_t size() const = 0;

            /**
             * @brief Use this to access the items inside the list.
             * @returns a reference to the item at the given index.
             */
            virtual TYPE &operator[](size_t index) = 0;

            /**
             * @brief Use this to access the items inside the list. Items cannot be modified!
             * @returns a const reference to the item at the given index.
             */
            virtual const TYPE &operator[](size_t index) const = 0;

            /**
             * @brief Slower but safer list access methode. Will roll back to 0 if index goes over end. Negative values start from end of list.
             * @returns a reference to the item at the given index.
             */
            virtual TYPE &operator()(int32_t index);

            /**
             * @brief Slower but safer list access methode. Will roll back to 0 if index goes over end. Negative values start from end of list.
             * @note Const function. Returned item cannot be modified.
             * @returns a reference to the item at the given index.
             */
            virtual const TYPE &operator()(int32_t index) const;

            /**
             * @brief   Will copy the items in the given list into the list. The number of items to be copied is the size of the smaller array.
             *          This is usually the size of the array being copied but can be limited by the copying array if its a fixed size and smaller.
             *          Not required to be implemented by child class but can improve performance by doing so.
             * @returns reference to this array.
             */
            virtual List<TYPE> &operator=(const List<TYPE> &listB);

            /**
             * @brief   Will copy the items in the given list into the list. The number of items to be copied is the size of the smaller array.
             *          As this function is receiving a list with a different data type. The data must be casted and the looping is slower. Are you sure about this?
             * @returns reference to this array.
             */
            template <typename TYPE2>
            List<TYPE> &operator=(const List<TYPE2> &listB);
        };

        /**
         * @brief A class used for signifying an empty list. 
         * @note Access operators will always return the same object.
         * @tparam TYPE 
         */
        template<typename TYPE>
        class EmptyList : public List<TYPE>
        {
        private:

            TYPE type_;

        public: 

            size_t size() const override
                { return 0; } 

            TYPE &operator[](size_t index) override 
            {
                return type_;
            }

            const TYPE &operator[](size_t index) const override
            {
                return type_;
            }

            TYPE &operator()(int32_t index) override
            {
                return type_;
            }

            const TYPE &operator()(int32_t index) const override
            {
                return type_;
            }

            List<TYPE> &operator=(const List<TYPE> &listB) override
            {
                return *this;
            }

        };

        template <typename TYPE>
        const List<TYPE> &List<TYPE>::empty() 
        {
            static EmptyList<TYPE> emptyList;
            return emptyList;
        }

        template <typename TYPE>
        TYPE &List<TYPE>::operator()(int32_t index)
        {
            auto len = size();
            if (index < 0)
                index = int32_t(len) + index;

            return (*this)[index % len];
        }

        template <typename TYPE>
        const TYPE &List<TYPE>::operator()(int32_t index) const
        {
            return (*this)(index);
        }

        template <typename TYPE>
        List<TYPE> &List<TYPE>::operator=(const List<TYPE> &listB)
        {

            for (size_t i = 0; i < size() && i < listB.size(); i++)
            {
                (*this)[i] = listB[i];
            }

            return *this;
        }

        template <typename TYPE>
        template <typename TYPE2>
        List<TYPE> &List<TYPE>::operator=(const List<TYPE2> &listB)
        {

            for (size_t i = 0; i < size() && i < listB.size(); i++)
            {
                (*this)[i] = static_cast<TYPE>(listB[i]);
            }

            return *this;
        }

    }

}

#endif