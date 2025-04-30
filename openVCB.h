/*
 * This is the primary header file for openVCB.
 */
#pragma once
#ifndef C8kWpReCttGxHsWkLLl1RDjAweb3HDua
#define C8kWpReCttGxHsWkLLl1RDjAweb3HDua

// Toggle experimental byte addressed VMem.
//#define OVCB_BYTE_ORIENTED_VMEM

#include "openVCB_Utils.hh"
#include "openVCB_Data.hh"

#if defined _MSC_VER
# define OVCB_CONSTEXPR constexpr __forceinline
# define OVCB_INLINE    __forceinline
# if !(defined __GNUC__ || defined __clang__ || defined __INTEL_COMPILER || defined __INTEL_LLVM_COMPILER)
#   pragma warning(disable: 5030)  // Unrecognized attribute
# endif
#elif defined __GNUC__ || defined __clang__
# define OVCB_CONSTEXPR __attribute__((__always_inline__)) constexpr inline
# define OVCB_INLINE    __attribute__((__always_inline__)) inline
#else
# define OVCB_CONSTEXPR constexpr inline
# define OVCB_INLINE    inline
#endif


namespace openVCB {
/****************************************************************************************/


/// <summary>
/// 8 bit state.
/// 7 bit type + 1 active bit
///
/// Add potential ink types here.
/// It is VERY important to keep components in pairs for on and off to
/// to make sure the values are aligned correctly.
/// </summary>
enum class Ink : uint8_t {
    None = 0,

    Trace  = 1,
    Read   = 2,
    Write  = 3,
    Cross  = 4,
    Buffer = 5,
    Or     = 6,
    Nand   = 7,
    Not    = 8,
    Nor    = 9,
    And    = 10,
    Xor    = 11,
    Xnor   = 12,
    Clock  = 13,
    Latch  = 14,
    Led    = 15,
    Bus    = 16,

    Filler     = 17,
    Annotation = 18,
    Tunnel     = 19,
    Mesh       = 20,

    Timer      = 21,
    Random     = 22,
    Breakpoint = 23,
    Wireless0  = 24,
    Wireless1  = 25,
    Wireless2  = 26,
    Wireless3  = 27,

    _numTypes = 28,
    _inkOn    = 0x80,

    TraceOn  = Trace  | _inkOn,
    ReadOn   = Read   | _inkOn,
    WriteOn  = Write  | _inkOn,
    BufferOn = Buffer | _inkOn,
    OrOn     = Or     | _inkOn,
    NandOn   = Nand   | _inkOn,
    NotOn    = Not    | _inkOn,
    NorOn    = Nor    | _inkOn,
    AndOn    = And    | _inkOn,
    XorOn    = Xor    | _inkOn,
    XnorOn   = Xnor   | _inkOn,
    ClockOn  = Clock  | _inkOn,
    LatchOn  = Latch  | _inkOn,
    LedOn    = Led    | _inkOn,
    BusOn    = Bus    | _inkOn,

    InvalidCross      = Cross      | _inkOn,
    InvalidFiller     = Filler     | _inkOn,
    InvalidAnnotation = Annotation | _inkOn,
    InvalidTunnel     = Tunnel     | _inkOn,
    InvalidMesh       = Mesh       | _inkOn,

    TimerOn      = Timer      | _inkOn,
    RandomOn     = Random     | _inkOn,
    BreakpointOn = Breakpoint | _inkOn,
    Wireless0On  = Wireless0  | _inkOn,
    Wireless1On  = Wireless1  | _inkOn,
    Wireless2On  = Wireless2  | _inkOn,
    Wireless3On  = Wireless3  | _inkOn,
};


/**
 * @brief 8 bit state. 7 bit type + 1 active bit.
 * @note
 * - Add new logic behavior here. All inks define their logic from this.
 * - This is seperate from Ink to simplify switch logic
 */
enum class Logic : uint8_t {
    None = 0,

    NonZero    = 1,
    Zero       = 2,
    Xor        = 3,
    Xnor       = 4,
    Latch      = 5,
    Clock      = 6,
    Timer      = 7,
    Random     = 8,
    Breakpoint = 9,

    _numTypes = 10,
    _logicOn  = 0x80,

    NonZeroOn    = NonZero    | _logicOn,
    ZeroOn       = Zero       | _logicOn,
    XorOn        = Xor        | _logicOn,
    XnorOn       = Xnor       | _logicOn,
    LatchOn      = Latch      | _logicOn,
    ClockOn      = Clock      | _logicOn,
    TimerOn      = Timer      | _logicOn,
    RandomOn     = Random     | _logicOn,
    BreakpointOn = Breakpoint | _logicOn,
};


/*--------------------------------------------------------------------------------------*/


struct InkState {
    int16_t activeInputs; // Number of active inputs
    bool    visited;      // Flags for traversal
    Logic   logic;        // Current logic state
};

struct SparseMat {
    int  n;    // Size of the matrix
    int  nnz;  // Number of non-zero entries
    int *ptr;  // CSC sparse matrix ptr
    int *rows; // CSC sparse matrix rows
};

// This represents a pixel with metadata as well as simulation ink type
// Ths is for inks that have variants which do not affect the simulation:
// i.e. colored traces.
struct InkPixel {
    Ink     ink;
    int16_t meta;
};

// This is for asyncronous exchange of data.
// for stuff like audio and signal scopes
struct InstrumentBuffer {
    InkState *buffer;
    int32_t   bufferSize;
    int32_t   idx;
};

struct SimulationResult {
    int32_t numTicksProcessed;
    bool    breakpoint;
};

struct LatchInterface {
    glm::ivec2 pos;     // Coordinates of the LSB latch.
    glm::ivec2 stride;  // Space between each latch (both x and y).
    glm::ivec2 size;    // Physical ize of the latch in squares as (x,y) pair.
    int        numBits; // Value always 32 or lower
    int        gids[64];
};

/*--------------------------------------------------------------------------------------*/
} // namespace openVCB

#include "openVCB_Helpers.hh"

namespace openVCB {
/*--------------------------------------------------------------------------------------*/


/**
 * \brief Main data structure for a compiled simulation.
 */
class Project
{
  public:
    // This remains null if VMem is not actually used.
    VMemWrapper vmem         = nullptr;
    uint32_t    vmemSize     = 0;
    uint32_t    lastVMemAddr = 0;
    uint64_t    tickNum      = 0;

    LatchInterface vmAddr    = {{0,0}, {0,0}, {0,0}, -1, {}};
    LatchInterface vmData    = {{0,0}, {0,0}, {0,0}, -1, {}};
    std::string    assembly  = {};
    int32_t        height    = 0;
    int32_t        width     = 0;
    int32_t        numGroups = 0;

    // Event queue
    uint32_t   qSize            = 0;
    int32_t   *updateQ[2]       = {nullptr, nullptr};
    int16_t   *lastActiveInputs = nullptr;
    bool       states_is_native = false;
    bool       vmemIsBytes      = false;

    RandomBitProvider random;
    StringArray      *error_messages  = nullptr;

    // An image containing component indices.
    uint8_t  *originalImage = nullptr;
    InkPixel *image         = nullptr;
    int32_t  *indexImage    = nullptr;
    int32_t  *decoration[3] = {nullptr, nullptr, nullptr}; // on / off / unknown

    uint32_t ledPalette[16] = {
        0x323841, 0xFFFFFF,
        0xFF0000, 0x00FF00, 0x0000FF,
        0xFF0000, 0x00FF00, 0x0000FF,
        0xFF0000, 0x00FF00, 0x0000FF,
        0xFF0000, 0x00FF00, 0x0000FF,
        0xFF0000, 0x00FF00
    };

    ClockCounter tickClock;
    TimerCounter realtimeClock;

    // Adjacentcy matrix.
    // By default, the indices from ink groups first and then component groups.
    SparseMat writeMap = {0, 0, nullptr, nullptr};

    // Stores the logic states of each group.
    InkState *states = nullptr;

    // Stores the actual ink type of each group.
    Ink *stateInks = nullptr;

    // Map of symbols during assembleVmem().
    std::map<std::string, uint32_t> assemblySymbols;
    std::map<uint32_t, uint32_t>    lineNumbers;
    std::vector<InstrumentBuffer>   instrumentBuffers;

    //---------------------------------------------------------------------------------

    explicit Project(int64_t seed, bool vmemIsBytes);
    ~Project();

    // Clang complains unless *all* possible constructors are defined.
    Project(Project const &)                = delete;
    Project &operator=(Project const &)     = delete;
    Project(Project &&) noexcept            = delete;
    Project &operator=(Project &&) noexcept = delete;

    //---------------------------------------------------------------------------------

    // Builds a project from an image. Remember to configure VMem.
    void readFromVCB(std::string const &filePath);

    // Decode base64 data from clipboard, then process logic data.
    bool readFromBlueprint(std::string clipboardData);

    // Decompress zstd logic data to an image.
    bool processLogicData(std::vector<uint8_t> const &logicData, int32_t headerSize);

    // Decompress zstd decoration data to an image.
    static void processDecorationData(std::vector<uint8_t> const &decorationData,
                                      int32_t                   *&decoData);

    // Samples the ink at a pixel. Returns ink and group id.
    ND std::pair<Ink, int32_t> sample(glm::ivec2 pos) const;

    // Assemble the vmem from this->assembly.
    void assembleVmem(char *errp = nullptr, size_t errSize = 256);

    // Dump vmem contents to file.
    void dumpVMemToText(std::string const &p) const;

    // Toggles the latch at position.
    // Does nothing if it's not a latch.
    void toggleLatch(glm::ivec2 pos);

    // Toggles the latch at position.
    // Does nothing if it's not a latch.
    void toggleLatch(int32_t gid);

    // Preprocesses the image into the simulation format.
    void preprocess();

    // Methods to add and remove breakpoints.
    void addBreakpoint(int32_t gid);
    void removeBreakpoint(int32_t gid);

    /// Advances the simulation by n ticks
    [[gnu::hot]]
    SimulationResult tick(int32_t numTicks = 1, int64_t maxEvents = INT64_MAX);

    //---------------------------------------------------------------------------------

  private:
    [[gnu::hot]]
    ND bool resolve_state(SimulationResult &res, InkState curInk, int lastInputs, bool lastActive);

    [[gnu::hot]]
    bool tryEmit(int32_t gid);

    void handleWordVMemTick();
#ifdef OVCB_BYTE_ORIENTED_VMEM
      void handleByteVMemTick();
#endif

    OVCB_INLINE bool GetRandomBit() { return static_cast<bool>(random()); }
};


/****************************************************************************************/
} // namespace openVCB
#endif
