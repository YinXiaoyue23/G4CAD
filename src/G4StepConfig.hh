#ifndef G4StepConfig_HH
#define G4StepConfig_HH

// Central, read-once snapshot of all G4CAD_* environment switches.
//
// Previously ~13 std::getenv() calls were scattered across G4StepSolid.cc and
// G4StepSolidImpl.cc (construction, AlgoCache build, per-query). This struct reads
// them all ONCE (thread-safe, at first access) into one place, and every call site
// references G4StepConfig::Get().<field>. Behaviour is byte-identical to the old
// scattered reads (same defaults, same parsing) — env is still the single source of
// truth; this only centralises WHERE it is read. Internal header (not installed,
// not part of the public G4StepSolid API); construction-time override can be layered
// on later without touching call sites.

#include <string>
#include <cstdlib>

struct G4StepConfig {
    // --- debug query recorder ---
    std::string recordPath;         // G4CAD_RECORD           CSV path ("" = recorder off)
    int         recordEvent = -1;   // G4CAD_RECORD_EVENT     record only this event id (-1 = all)

    // --- behaviour gates ---
    bool   insideForceOCCT = false; // G4CAD_INSIDE_FORCE_OCCT=1  force OCCT classifier for Inside()
    bool   timing          = false; // G4CAD_TIMER            per-call timing report at exit
    bool   insideSelfcheck = false; // G4CAD_INSIDE_SELFCHECK analytic-vs-OCCT Inside self-check
    bool   perSolidTol     = false; // G4CAD_PERSOLID_TOL=1   per-solid tolerance band

    // --- CT-TRIM (UV trim model) ---
    bool   uvTrimOff       = false; // G4CAD_UVTRIM_OFF       disable CT-TRIM (A/B vs CT1)
    bool   uvTrimVerbose   = false; // G4CAD_UVTRIM_VERBOSE   CT-TRIM build summary
    bool   uvTrimDebug     = false; // G4CAD_UVTRIM_DEBUG     CT-TRIM per-face debug
    double uvTrimBand      = 0.02;  // G4CAD_UVTRIM_BAND      CT-TRIM boundary band (UV units)
    bool   uvTrimNoPoly    = false; // G4CAD_UVTRIM_NOPOLY    disable seam-aware polygon model

    // --- diagnostics ---
    bool   parityDirSel    = false; // G4CAD_PARITY_DIRSEL    print direction-selection timing
    bool   tolVerbose      = false; // G4CAD_TOL_VERBOSE      per-solid tolerance print

    // Single shared instance, initialised once from the environment.
    static const G4StepConfig& Get() {
        static const G4StepConfig cfg = [] {
            G4StepConfig c;
            auto present = [](const char* k) { return std::getenv(k) != nullptr; };
            if (const char* p = std::getenv("G4CAD_RECORD"))           c.recordPath = p;
            if (const char* e = std::getenv("G4CAD_RECORD_EVENT"))     if (*e) c.recordEvent = std::atoi(e);
            if (const char* f = std::getenv("G4CAD_INSIDE_FORCE_OCCT"))c.insideForceOCCT = (f[0] == '1');
            c.timing          = present("G4CAD_TIMER");
            c.insideSelfcheck = present("G4CAD_INSIDE_SELFCHECK");
            if (const char* t = std::getenv("G4CAD_PERSOLID_TOL"))     c.perSolidTol = (t[0] == '1');
            c.uvTrimOff       = present("G4CAD_UVTRIM_OFF");
            c.uvTrimVerbose   = present("G4CAD_UVTRIM_VERBOSE");
            c.uvTrimDebug     = present("G4CAD_UVTRIM_DEBUG");
            if (const char* b = std::getenv("G4CAD_UVTRIM_BAND"))      c.uvTrimBand = std::atof(b);
            c.uvTrimNoPoly    = present("G4CAD_UVTRIM_NOPOLY");
            c.parityDirSel    = present("G4CAD_PARITY_DIRSEL");
            c.tolVerbose      = present("G4CAD_TOL_VERBOSE");
            return c;
        }();
        return cfg;
    }
};

#endif // G4StepConfig_HH
