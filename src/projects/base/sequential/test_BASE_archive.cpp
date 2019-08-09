// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Test for unique_ptr in Chrono archives. We use a trick / work-around here
// because of current ChArchive lack of support. 
//
// =============================================================================

#include "chrono/serialization/ChArchive.h"
#include "chrono/serialization/ChArchiveBinary.h"
#include "chrono/serialization/ChArchiveAsciiDump.h"
#include "chrono/serialization/ChArchiveJSON.h"
#include "chrono/serialization/ChArchiveXML.h"
#include "chrono/serialization/ChArchiveExplorer.h"

#include "chrono/core/ChClassFactory.h"
#include "chrono/core/ChVector.h"

#include "chrono_thirdparty/filesystem/path.h"

// =============================================================================

namespace chrono {

// Place these classes in the 'chrono' namespace (to use class versions)

class Object {
  public:
    Object() {}
    ~Object() {}

    void SetVec(const ChVector<>& c) { m_vec = c; }
    const ChVector<>& GetVec() const { return m_vec; }

    // Set the vector maintained as a unique_ptr. Transfers ownership.
    void SetVecU(std::unique_ptr<ChVector<>>&& v) { m_vecU = std::move(v); }

    // Get a handle on the wrapped pointer for possible modification.
    // Use a raw pointer (and not a reference) because the vector may not exist.
    ChVector<>* GetVecU() const { return m_vecU.get(); }

    void Print() {
        GetLog() << "   v: " << m_vec << "\n";
        GetLog() << "  vU: ";
        if (m_vecU) {
            GetLog() << *m_vecU.get() << "\n";
        } else {
            GetLog() << " empty\n";
        }
    }

    // We must play special tricks here because a unique_ptr cannot be simply copied.
    // Instead we archive the wrapped pointer. On deserialization, we read the raw pointer
    // and use it to replace the object managed by the unique_ptr.

    void ArchiveOUT(ChArchiveOut& marchive) {
        marchive.VersionWrite<Object>();
        marchive << CHNVP(m_vec);
        marchive << CHNVP(m_vecU.get(), "m_vecU_ptr");
    }

    void ArchiveIN(ChArchiveIn& marchive) {
        int version = marchive.VersionRead<Object>();
        marchive >> CHNVP(m_vec);
        {
            ChVector<>* vecU_ptr;
            marchive >> CHNVP(vecU_ptr, "m_vecU_ptr");
            m_vecU.reset(vecU_ptr);
        }
    }

    ChVector<> m_vec;
    std::unique_ptr<ChVector<>> m_vecU;
};
CH_CLASS_VERSION(Object, 5)

class Container {
  public:
    Container(std::shared_ptr<Object> A, std::shared_ptr<Object> B) : m_A(A), m_B(B) {}

    void Print() {
        GetLog() << "Object A\n";
        m_A->Print();
        GetLog() << "Object B\n";
        m_B->Print();
    }

    void ArchiveOUT(ChArchiveOut& marchive) {
        marchive.VersionWrite<Container>();
        marchive << CHNVP(m_A);
        marchive << CHNVP(m_B);
    }

    void ArchiveIN(ChArchiveIn& marchive) {
        int version = marchive.VersionRead<Container>();
        marchive >> CHNVP(m_A);
        marchive >> CHNVP(m_B);
    }

    std::shared_ptr<Object> m_A;
    std::shared_ptr<Object> m_B;
};
CH_CLASS_VERSION(Container, 5)

}  // namespace chrono

// =============================================================================

using namespace chrono;

void write_archive(ChArchiveOut& archive) {
    auto A = chrono_types::make_shared<Object>();
    A->SetVec(ChVector<>(11, 12, 13));
    A->SetVecU(std::make_unique<ChVector<>>(ChVector<>(1, 2, 3)));

    // B with zero vector and empty unique_ptr.
    auto B = chrono_types::make_shared<Object>();

    auto C = chrono_types::make_shared<Container>(A, B);
    C->ArchiveOUT(archive);
}

void read_archive(ChArchiveIn& archive) {
    auto A = chrono_types::make_shared<Object>();
    auto B = chrono_types::make_shared<Object>();

    auto C = chrono_types::make_shared<Container>(A, B);
    C->ArchiveIN(archive);
    C->Print();
}

// =============================================================================

int main(int argc, char* argv[]) {
    std::string dir_name = "../ARCHIVE_TEST";

    if (!filesystem::create_directory(filesystem::path(dir_name))) {
        std::cout << "Error creating directory " << dir_name << std::endl;
        return 1;
    }

    {
        std::string file_name = dir_name + "/archive.txt";
        ChStreamOutAsciiFile file_out(file_name.c_str());
        ChArchiveAsciiDump archive_out(file_out);
        write_archive(archive_out);
    }

    {
        std::string file_name = dir_name + "/archive.dat";
        ChStreamOutBinaryFile file_out(file_name.c_str());
        ChArchiveOutBinary archive_out(file_out);
        write_archive(archive_out);
    }

    {
        GetLog() << "\nDESERIALIZATION FROM BINARY\n";
        std::string file_name = dir_name + "/archive.dat";
        ChStreamInBinaryFile file_in(file_name.c_str());
        ChArchiveInBinary archive_in(file_in);
        read_archive(archive_in);
    }

    {
        std::string file_name = dir_name + "/archive.json";
        ChStreamOutAsciiFile file_out(file_name.c_str());
        ChArchiveOutJSON archive_out(file_out);
        write_archive(archive_out);
    }

    {
        GetLog() << "\nDESERIALIZATION FROM JSON\n";
        std::string file_name = dir_name + "/archive.json";
        ChStreamInAsciiFile file_in(file_name.c_str());
        ChArchiveInJSON archive_in(file_in);
        read_archive(archive_in);
    }

    return 0;
}
