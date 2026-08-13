#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace VIO::dcreg_shadow::sha256 {

using Digest = std::array<std::uint8_t, 32>;
using Digest128 = std::array<std::uint8_t, 16>;

Digest hash(const std::uint8_t* data, std::size_t size);
Digest hash(const std::vector<std::uint8_t>& data);
Digest hash(const std::string& data);
Digest128 prefix128(const Digest& digest);
std::string hex(const Digest& digest);
std::string hex(const Digest128& digest);

}  // namespace VIO::dcreg_shadow::sha256
