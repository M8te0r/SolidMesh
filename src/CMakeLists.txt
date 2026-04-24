file(GLOB_RECURSE POLYHEDRA_SOURCES CONFIGURE_DEPENDS
    "*.cpp"
    "*.cc"
    "*.cxx"
)

target_sources(polyhedra-core PRIVATE ${POLYHEDRA_SOURCES})
