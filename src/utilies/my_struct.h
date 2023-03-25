#pragma once
#include <iostream>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
namespace lvio_2d
{
    template <typename T>
    class my_2d_vec
    {
    public:
        my_2d_vec(int h_, int w_)
        {
            w = w_;
            h = h_;
            memory_ = static_cast<T **>(malloc(h_ * w_ * sizeof(T *)));
            int total = h * w;
            int n = total / 64 + 1;
            has_init = static_cast<uint64_t *>(malloc(n * sizeof(uint64_t)));
            memset(has_init, 0, n * sizeof(uint64_t));
            has_free = false;
        }
        ~my_2d_vec()
        {
            destroy();
        }
        T &operator()(int r, int c)
        {
            int index = r * w + c;
            uint64_t m = index / 64;
            uint64_t n = index % 64;
            uint64_t mask = (uint64_t(1) << n);
          
            if (!(has_init[m] & mask))
            {
                has_init[m] |= mask;
                memory_[index] = new T;
                has_create.push_back(index);
            }

            return *(memory_[index]);
        }
        void destroy()
        {
            if (has_free)
                return;
            for (int i = 0; i < has_create.size(); i++)
                delete memory_[has_create[i]];

            free(memory_);
            free(has_init);
            has_free = true;
        }

    private:
        int w, h;
        T **memory_;
        uint64_t *has_init;
        std::vector<uint64_t> has_create;
        bool has_free;
    };
} // namespace lvio_2d
