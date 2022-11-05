/*
 * Copyright (c) 2022 Shahir Mowlaei
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial
 * portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT.  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include <mutex>
#include <queue>
#include <thread>
#include <barrier>

#include "support.hpp"


using ThreadCntType = std::uint32_t;

/*
 * see 'Spawner::spawn' or 'Pooler::pool' for the
 * pattern invoked by each (intermediate) constant
 */
enum class CallPattern
{
    FBDNL, // 0 (forbidden lower bound)
    
    FUNCT, // 1 : func(args[idx]...)
    FNIDX, // 2 : func(idx, args...)
    
    FNOBJ, // 3 :   func[idx] (args...)
    FNOBP, // 4 : (*func[idx])(args...)

    FBDNU  // 5 (forbidden upper bound)
};


class Spawner
{
public:

    explicit
    Spawner(ThreadCntType ntd) noexcept : ntd { ntd } {};
    
     Spawner() = delete;
    ~Spawner() = default;

    Spawner(Spawner &  src) = delete;
    Spawner(Spawner && src) = delete;

    Spawner & operator=(Spawner &  rhs) = delete;
    Spawner & operator=(Spawner && rhs) = delete;

    template<CallPattern pattern, typename F, typename... Tn>
    void
    spawn(std::queue<IdxType> & que, F && func, Tn && ... args);
    
protected:

    ThreadCntType const ntd;
    std::vector<std::thread> tds;
};


template<CallPattern pattern, typename F, typename... Tn>
void
Spawner::spawn(std::queue<IdxType> & que, F && func, Tn && ... args)
{
    static_assert((pattern > CallPattern::FBDNL) and (pattern < CallPattern::FBDNU));
    
    std::mutex mu;

    auto const lambda
    {
        [& mu, & que, & func, & args...]
        {
            std::unique_lock lock { mu };

            while (!que.empty())
            {
                auto idx { que.front() };
                que.pop();

                lock.unlock();

                if constexpr (pattern == CallPattern::FUNCT)
                    std::forward<F>(func)(std::forward<Tn>(args)[idx]...);
                if constexpr (pattern == CallPattern::FNIDX)
                    std::forward<F>(func)(idx, std::forward<Tn>(args)...);
                if constexpr (pattern == CallPattern::FNOBJ)
                    std::forward<F>(func)[idx](std::forward<Tn>(args)...);
                if constexpr (pattern == CallPattern::FNOBP)
                    (* std::forward<F>(func)[idx])(std::forward<Tn>(args)...);

                lock.lock();
            }
        }
    };
        
    for (ThreadCntType i {}; i < ntd; i++)
        tds.emplace_back(std::thread { lambda });

    for (auto & td : tds)
        td.join();

    tds.clear();
}


template<typename B>
class Pooler
{
public:

    Pooler(ThreadCntType ntd, std::barrier<B> & barry) noexcept : ntd { ntd } , barry { barry } {};
    
     Pooler() = delete;
    ~Pooler() = default;

    Pooler(Pooler &  src) = delete;
    Pooler(Pooler && src) = delete;

    Pooler & operator=(Pooler &  rhs) = delete;
    Pooler & operator=(Pooler && rhs) = delete;

    void shutdown()
        {
            // the call to 'barrier::arrive_and_wait' is
            // absent here as client code is expected to
            // adhere to a /parity shift/ pattern:

            // /* parity shift pattern */
            //
            // Pooler { numThreads, barrier { numThreads + 1 } }
            // Pooler::poolMethod();
            // barrier::arrive_and_wait();
            // do {
            //     manifestCompletnFunction();
            //     barrier::arrive_and_wait();
            //     // maintenance();
            //     barrier::arrive_and_wait();
            // } while (!triggerShutdown);
            // Pooler::shutdown();
            
            shutdownFlag = true;
            barry.arrive_and_wait();

            for (auto & td : tds)
                td.join();
            tds.clear();

            shutdownFlag = false;
        }

    template<CallPattern pattern, typename F, typename... Tn>
    void
    pool(std::queue<IdxType> & que, F && func, Tn && ... args);

private:
    
    ThreadCntType const ntd;

    std::atomic<bool> shutdownFlag { false };

    std::barrier<B> & barry;

    std::mutex mu;
    
    std::vector<std::thread> tds;
};


template <typename B>
template <CallPattern pattern, typename F, typename... Tn>
void
Pooler<B>::pool(std::queue<IdxType> & que, F && func, Tn && ... args)
{
    static_assert((pattern > CallPattern::FBDNL) and (pattern < CallPattern::FBDNU));

    auto const lambda
    {
        [this, & que, & func, & args...]
        {
             while (!shutdownFlag)
            {
                std::unique_lock lock { mu };

                while (!que.empty())
                {
                    auto idx { que.front() };
                    que.pop();

                    lock.unlock();

                    if constexpr (pattern == CallPattern::FUNCT)
                        std::forward<F>(func)(std::forward<Tn>(args)[idx]...);
                    if constexpr (pattern == CallPattern::FNIDX)
                        std::forward<F>(func)(idx, std::forward<Tn>(args)...);
                    if constexpr (pattern == CallPattern::FNOBJ)
                        std::forward<F>(func)[idx](std::forward<Tn>(args)...);
                    if constexpr (pattern == CallPattern::FNOBP)
                        (* std::forward<F>(func)[idx])(std::forward<Tn>(args)...);

                    lock.lock();
                }

                lock.unlock();

                barry.arrive_and_wait();
                barry.arrive_and_wait();
            }
        }
    };
        
    for (ThreadCntType i {}; i < ntd; i++)
        tds.emplace_back(std::thread { lambda });
}


