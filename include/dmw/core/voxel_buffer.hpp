#pragma once
#include <array>
#include <atomic>
#include <optional>

namespace dmw::core
{
/** Anneau SPSC lock-free (producteur : ROS, consommateur : réseau). */
template<std::size_t N>
class VoxelBuffer
{
    static_assert((N & (N-1)) == 0, "N must be power of 2");
public:
    bool push(const Voxel& v) noexcept
    {
        auto h = head_.load(std::memory_order_relaxed);
        auto n = (h + 1) & mask_;
        if (n == tail_.load(std::memory_order_acquire))
            return false;                      // plein
        data_[h] = v;
        head_.store(n, std::memory_order_release);
        return true;
    }
    /** Pop jusqu'à 'out.capacity()' voxels. Renvoie combien ont été copiés. */
    template<typename Vec>
    std::size_t pop_bulk(Vec& out) noexcept
    {
        auto t = tail_.load(std::memory_order_relaxed);
        auto h = head_.load(std::memory_order_acquire);
        std::size_t avail = (h + N - t) & mask_;
        std::size_t n = std::min(avail, out.capacity());
        for (std::size_t i=0;i<n;++i)
            out.emplace_back(data_[(t + i) & mask_]);
        tail_.store((t + n) & mask_, std::memory_order_release);
        return n;
    }
private:
    static constexpr std::size_t mask_ = N - 1;
    std::array<Voxel,N> data_{};
    std::atomic<std::size_t> head_{0};
    std::atomic<std::size_t> tail_{0};
};
} // namespace dmw::core
