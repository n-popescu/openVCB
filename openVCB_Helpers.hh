#pragma once
#ifndef G1Gjjm08JMgUZALRqEUy6y5CngZGIr6L5gKrj321mPkr5Jds
#define G1Gjjm08JMgUZALRqEUy6y5CngZGIr6L5gKrj321mPkr5Jds

#include "openVCB.h"

#if defined _MSC_VER
# if !(defined __GNUC__ || defined __clang__ || defined __INTEL_COMPILER || defined __INTEL_LLVM_COMPILER)
#   pragma warning(disable: 5030)  // Unrecognized attribute
#endif
#endif


namespace openVCB {
/****************************************************************************************/


/*
 * Hideous boilerplate garbage to make using `enum class` objects less infuriatingly
 * tedious and "cast-tastic". This nonsense takes up less space on the page with the
 * long lines than if properly formatted, which is a win in my book.
 */

template <typename T> concept Integral = std::is_integral<T>::value;

ND OVCB_CONSTEXPR Logic operator>>(Logic val,  uint  n)    { return static_cast<Logic>(static_cast<uint>(val) >> n); }
ND OVCB_CONSTEXPR Logic operator<<(Logic val,  uint  n)    { return static_cast<Logic>(static_cast<uint>(val) << n); }
ND OVCB_CONSTEXPR Logic operator& (Logic val1, uint  val2) { return static_cast<Logic>(static_cast<uint>(val1) & val2); }
ND OVCB_CONSTEXPR Logic operator| (Logic val1, uint  val2) { return static_cast<Logic>(static_cast<uint>(val1) | val2); }
ND OVCB_CONSTEXPR Logic operator& (Logic val1, Logic val2) { return static_cast<Logic>(static_cast<uint>(val1) & static_cast<uint>(val2)); }
ND OVCB_CONSTEXPR Logic operator| (Logic val1, Logic val2) { return static_cast<Logic>(static_cast<uint>(val1) | static_cast<uint>(val2)); }
ND OVCB_CONSTEXPR Logic operator~ (Logic val)              { return static_cast<Logic>(~static_cast<uint>(val)); }

template <Integral T>
ND OVCB_CONSTEXPR bool operator==(Logic op1, T op2)
{
    return op1 == static_cast<Logic>(op2);
}


ND OVCB_CONSTEXPR Ink operator>>(Ink val,  uint n)    { return static_cast<Ink>(static_cast<uint>(val) >> n); }
ND OVCB_CONSTEXPR Ink operator<<(Ink val,  uint n)    { return static_cast<Ink>(static_cast<uint>(val) << n); }
ND OVCB_CONSTEXPR Ink operator& (Ink val1, uint val2) { return static_cast<Ink>(static_cast<uint>(val1) & val2); }
ND OVCB_CONSTEXPR Ink operator| (Ink val1, uint val2) { return static_cast<Ink>(static_cast<uint>(val1) | val2); }
ND OVCB_CONSTEXPR Ink operator& (Ink val1, Ink  val2) { return static_cast<Ink>(static_cast<uint>(val1) & static_cast<uint>(val2)); }
ND OVCB_CONSTEXPR Ink operator| (Ink val1, Ink  val2) { return static_cast<Ink>(static_cast<uint>(val1) | static_cast<uint>(val2)); }
ND OVCB_CONSTEXPR int operator+ (Ink val1, Ink  val2) { return static_cast<int>(val1) + static_cast<int>(val2); }
ND OVCB_CONSTEXPR int operator- (Ink val1, Ink  val2) { return static_cast<int>(val1) - static_cast<int>(val2); }
ND OVCB_CONSTEXPR int operator+ (Ink val1, int  val2) { return static_cast<int>(val1) + val2; }
ND OVCB_CONSTEXPR int operator- (Ink val1, int  val2) { return static_cast<int>(val1) - val2; }
ND OVCB_CONSTEXPR Ink operator~ (Ink val)             { return static_cast<Ink>(~static_cast<uint>(val)); }

template <Integral T>
ND OVCB_CONSTEXPR bool operator==(Ink op1, T op2)
{
    return op1 == static_cast<Ink>(op2);
}

ND OVCB_CONSTEXPR Ink   operator|(Ink val1, Logic val2) { return static_cast<Ink>  (static_cast<uint>(val1) | static_cast<uint>(val2)); }
ND OVCB_CONSTEXPR Ink   operator&(Ink val1, Logic val2) { return static_cast<Ink>  (static_cast<uint>(val1) & static_cast<uint>(val2)); }
ND OVCB_CONSTEXPR Logic operator|(Logic val1, Ink val2) { return static_cast<Logic>(static_cast<uint>(val1) | static_cast<uint>(val2)); }
ND OVCB_CONSTEXPR Logic operator&(Logic val1, Ink val2) { return static_cast<Logic>(static_cast<uint>(val1) & static_cast<uint>(val2)); }


constexpr bool operator==(InkPixel const &first, InkPixel const &second) noexcept
{
    return first.ink == second.ink && first.meta == second.meta;
}

/*--------------------------------------------------------------------------------------*/
/* Helper routines for Logic objects. */

/**
 * \brief Sets the ink (logic) type to be as specified by state.
 * \param logic The Ink (logic) value to modify.
 * \param state Should be 0 to turn off, 1 to turn on.
 * \return The modified value.
 */
ND OVCB_CONSTEXPR Logic SetOn(Logic logic, unsigned state)
{
    return (logic & ~Logic::_logicOn) | (state << 7);
}

// Sets the ink type to be on
ND OVCB_CONSTEXPR Logic SetOn(Logic logic)
{
    return logic | Logic::_logicOn;
}

// Sets the ink type to be off
ND OVCB_CONSTEXPR Logic SetOff(Logic logic)
{
    return logic & ~Logic::_logicOn;
}

// Gets the ink active state
ND OVCB_CONSTEXPR bool IsOn(Logic logic)
{
    return static_cast<bool>(logic & Logic::_logicOn);
}

/*--------------------------------------------------------------------------------------*/
/* Helper routines for Ink objects. */

/**
 * \brief Sets the ink type to be as specified by state.
 * \param ink The Ink value to modify.
 * \param state Should be 0 to turn off, 1 to turn on.
 * \return The modified value.
 */
ND OVCB_CONSTEXPR Ink SetOn(Ink ink, unsigned state)
{
    return (ink & ~Ink::_inkOn) | (state << 7);
}

// Sets the ink type to be on.
ND OVCB_CONSTEXPR Ink SetOn(Ink ink)
{
    return ink | Ink::_inkOn;
}

// Sets the ink type to be off
ND OVCB_CONSTEXPR Ink SetOff(Ink ink)
{
    return ink & ~Ink::_inkOn;
}

// Gets the ink active state
ND OVCB_CONSTEXPR bool IsOn(Ink ink)
{
    return static_cast<bool>(ink & Ink::_inkOn);
}


// Gets the logic type of said ink
// Define the logics of each ink here.
inline Logic
inkLogicType(Ink ink)
{
    ink = SetOff(ink);

    switch (ink) { // NOLINT(clang-diagnostic-switch-enum)
    case Ink::Latch:      return Logic::Latch;
    case Ink::Clock:      return Logic::Clock;
    case Ink::Random:     return Logic::Random;
    case Ink::Timer:      return Logic::Timer;
    case Ink::Breakpoint: return Logic::Breakpoint;

    case Ink::Xor:
        return Logic::Xor;
    case Ink::Xnor:
        return Logic::Xnor;

    case Ink::Not:
    case Ink::Nor:
    case Ink::And:
        return Logic::Zero;

    case Ink::Nand:
    case Ink::Or:
    case Ink::Buffer:
    case Ink::Trace:
    default:
        return Logic::NonZero;
    }
}

// Gets the string name of the ink
extern char const *getInkString(Ink ink);

/*--------------------------------------------------------------------------------------*/

/**
 * \brief Stores the vmem array. This union makes the byte-addressed variant of openVCB
 * simpler to implement and should have no runtime impact at all.
 * @note This level of obsessive nonsense is excessive even for me.
 */
union VMemWrapper
{
    uint32_t *i;
    uint8_t  *b;

#ifdef OVCB_BYTE_ORIENTED_VMEM
# define DEF_POINTER b
    using value_type = uint8_t;
#else
# define DEF_POINTER i
    using value_type = uint32_t;
#endif

    //---------------------------------------------------------------------------------

    VMemWrapper() noexcept = default;
    explicit VMemWrapper(uint32_t *ptr) noexcept
        : i(ptr)
    {}
    explicit VMemWrapper(uint8_t *ptr) noexcept
        : b(ptr)
    {}
    // ReSharper disable once CppNonExplicitConvertingConstructor
    VMemWrapper(std::nullptr_t) noexcept
        : DEF_POINTER(nullptr)
    {}

    //---------------------------------------------------------------------------------

    // Automatically use the default member when indexing.
    ND auto const &operator[](size_t idx) const noexcept { return DEF_POINTER[idx]; }
    ND auto       &operator[](size_t idx)       noexcept { return DEF_POINTER[idx]; }

    ND auto       &def()       noexcept { return DEF_POINTER; }
    ND auto const &def() const noexcept { return DEF_POINTER; }

    // Assign pointers to the default union member.
    VMemWrapper &operator=(value_type *ptr) noexcept
    {
        DEF_POINTER = ptr;
        return *this;
    }

    // Allow assigning nullptr directly.
    VMemWrapper &operator=(std::nullptr_t) noexcept
    {
        DEF_POINTER = nullptr;
        return *this;
    }

    // Allow comparing with nullptr directly.
    ND bool constexpr operator==(std::nullptr_t) const noexcept
    {
        return DEF_POINTER == nullptr;
    }

    // Allow checking whether the pointer null by placing it in a boolean
    // context just as if this were a bare pointer.
    ND explicit constexpr operator bool() const noexcept
    {
        return DEF_POINTER != nullptr;
    }

    ND uint32_t *word_at_byte(size_t offset) const noexcept
    {
        return reinterpret_cast<uint32_t *>(b + offset);
    }
};

#undef DEF_POINTER

/*--------------------------------------------------------------------------------------*/

class StringArray final
{
    char   **array_;
    uint32_t capacity_;
    uint32_t qty_ = 0;

    static constexpr size_t default_capacity = 32;

  public:
    explicit StringArray(uint32_t capacity = default_capacity)
        : array_(new char *[capacity]),
          capacity_(capacity)
    {}

    ~StringArray()
    {
        if (array_) {
            for (unsigned i = 0; i < qty_; ++i) {
                delete[] array_[i];
                array_[i] = nullptr;
            }
            delete[] array_;
            capacity_ = qty_ = 0;
            array_           = nullptr;
        }
    }

    StringArray(StringArray const &)                = delete;
    StringArray(StringArray &&) noexcept            = delete;
    StringArray &operator=(StringArray const &)     = delete;
    StringArray &operator=(StringArray &&) noexcept = delete;

    ND char *push_blank(size_t len)
    {
        if (qty_ + 1 == capacity_) {
            capacity_ += default_capacity;
            auto **tmp = new char *[capacity_];
            memmove(tmp, array_, qty_ * sizeof(char *));
            delete[] array_;
            array_ = tmp;
        }

        auto *str      = new char[len + 1];
        array_[qty_++] = str;
        return str;
    }

    void push(char const *orig, size_t len)
    {
        char *str = push_blank(len);
        memcpy(str, orig, len + 1);
    }

    void push(char const *orig)             { push(orig,        strlen(orig)); }
    void push(std::string const &orig)      { push(orig.data(), orig.size());  }
    void push(std::string_view const &orig) { push(orig.data(), orig.size());  }

    template <size_t N>
    void push(char const (&str)[N])
    {
        push(str, N);
    }

    ND bool     empty()    const { return qty_ == 0; }
    ND uint32_t size()     const { return qty_; }
    ND uint32_t capacity() const { return capacity_; }

    ND char const *const *const &data() const &
    {
        return array_;
    }

    ND char **&data() &
    {
        return array_;
    }
};

/*--------------------------------------------------------------------------------------*/

class RandomBitProvider
{
    using random_type = std::minstd_rand;

  public:
    RandomBitProvider()
        : RandomBitProvider(rnd_device_())
    {}

    explicit RandomBitProvider(uint32_t seed)
        : random_engine_(seed),
          current_(random_engine_())
    {}

    ND unsigned operator()()
    {
        if (avail_ > 0) {
            --avail_;
        } else {
            avail_   = num_bits - 1U;
            current_ = random_engine_();
        }

        unsigned ret = current_ & 1U;
        current_ >>= 1;
        return ret;
    }

  private:
    static constexpr int num_bits = sizeof(uint32_t) * CHAR_BIT;
    inline static std::random_device rnd_device_{};

    random_type random_engine_;
    uint32_t    current_;
    int         avail_ = num_bits;
};

/*--------------------------------------------------------------------------------------*/


class ClockCounter final
{
  public:
    ClockCounter()
        : periods_({1, 1})
    {}
    explicit ClockCounter(uint16_t period)
        : periods_({period, period})
    {}
    explicit ClockCounter(uint16_t low, uint16_t high)
        : periods_({low, high})
    {}

    bool tick()
    {
        if (++counter_ >= periods_[state_]) {
            state_   = state_ ? 0 : 1;
            counter_ = 0;
            return true;
        }
        return false;
    }

    ND bool is_zero() const { return counter_ == 0; }
    ND int  counter() const { return counter_; }

    void set_period(uint16_t period)
    {
        set_period(period, period);
    }

    void set_period(uint16_t low, uint16_t high)
    {
        periods_[0] = low;
        periods_[1] = high;
    }

    std::vector<int32_t> GIDs;

  private:
    std::array<uint16_t, 2> periods_;
    uint16_t counter_ = 0;
    uint8_t  state_   = 0;
};


class TimerCounter final
{
  public:
    explicit TimerCounter(uint32_t period = 1)
        : period_(period)
    {}

    bool tick()
    {
        if (++counter_ >= period_) [[unlikely]] {
            counter_ = 0;
            return true;
        }
        return false;
    }

    ND bool     is_zero() const { return counter_ == 0; }
    ND uint32_t counter() const { return counter_; }

    void set_period(uint32_t val) { period_ = val; }

    std::vector<int32_t> GIDs;

  private:
    uint32_t period_;
    uint32_t counter_ = 0;
};


/****************************************************************************************/
} // namespace openVCB
#endif
