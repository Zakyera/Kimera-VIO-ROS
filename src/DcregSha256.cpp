#include "kimera_vio_ros/DcregSha256.h"

#include <algorithm>
#include <array>
#include <iomanip>
#include <sstream>

namespace VIO::dcreg_shadow::sha256 {
namespace {

constexpr std::array<std::uint32_t, 64> kRoundConstants = {
    0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u,
    0x3956c25bu, 0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u,
    0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u,
    0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u,
    0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu,
    0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
    0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u,
    0xc6e00bf3u, 0xd5a79147u, 0x06ca6351u, 0x14292967u,
    0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u,
    0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
    0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u,
    0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
    0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u,
    0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu, 0x682e6ff3u,
    0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u,
    0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u};

std::uint32_t rotateRight(std::uint32_t value, unsigned shift) {
  return (value >> shift) | (value << (32u - shift));
}

void appendBigEndian64(std::vector<std::uint8_t>* bytes,
                       std::uint64_t value) {
  for (int shift = 56; shift >= 0; shift -= 8) {
    bytes->push_back(static_cast<std::uint8_t>(value >> shift));
  }
}

}  // namespace

Digest hash(const std::uint8_t* data, std::size_t size) {
  std::vector<std::uint8_t> padded;
  padded.reserve(size + 72u);
  if (data != nullptr && size != 0u) {
    padded.insert(padded.end(), data, data + size);
  }
  padded.push_back(0x80u);
  while ((padded.size() % 64u) != 56u) {
    padded.push_back(0u);
  }
  appendBigEndian64(&padded, static_cast<std::uint64_t>(size) * 8u);

  std::array<std::uint32_t, 8> state = {
      0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
      0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u};
  for (std::size_t offset = 0u; offset < padded.size(); offset += 64u) {
    std::array<std::uint32_t, 64> words{};
    for (std::size_t i = 0u; i < 16u; ++i) {
      const std::size_t at = offset + 4u * i;
      words[i] = (static_cast<std::uint32_t>(padded[at]) << 24u) |
                 (static_cast<std::uint32_t>(padded[at + 1u]) << 16u) |
                 (static_cast<std::uint32_t>(padded[at + 2u]) << 8u) |
                 static_cast<std::uint32_t>(padded[at + 3u]);
    }
    for (std::size_t i = 16u; i < words.size(); ++i) {
      const std::uint32_t s0 = rotateRight(words[i - 15u], 7u) ^
                               rotateRight(words[i - 15u], 18u) ^
                               (words[i - 15u] >> 3u);
      const std::uint32_t s1 = rotateRight(words[i - 2u], 17u) ^
                               rotateRight(words[i - 2u], 19u) ^
                               (words[i - 2u] >> 10u);
      words[i] = words[i - 16u] + s0 + words[i - 7u] + s1;
    }

    std::uint32_t a = state[0];
    std::uint32_t b = state[1];
    std::uint32_t c = state[2];
    std::uint32_t d = state[3];
    std::uint32_t e = state[4];
    std::uint32_t f = state[5];
    std::uint32_t g = state[6];
    std::uint32_t h = state[7];
    for (std::size_t i = 0u; i < words.size(); ++i) {
      const std::uint32_t sum1 = rotateRight(e, 6u) ^
                                 rotateRight(e, 11u) ^
                                 rotateRight(e, 25u);
      const std::uint32_t choose = (e & f) ^ ((~e) & g);
      const std::uint32_t temp1 =
          h + sum1 + choose + kRoundConstants[i] + words[i];
      const std::uint32_t sum0 = rotateRight(a, 2u) ^
                                 rotateRight(a, 13u) ^
                                 rotateRight(a, 22u);
      const std::uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
      const std::uint32_t temp2 = sum0 + majority;
      h = g;
      g = f;
      f = e;
      e = d + temp1;
      d = c;
      c = b;
      b = a;
      a = temp1 + temp2;
    }
    state[0] += a;
    state[1] += b;
    state[2] += c;
    state[3] += d;
    state[4] += e;
    state[5] += f;
    state[6] += g;
    state[7] += h;
  }

  Digest result{};
  for (std::size_t i = 0u; i < state.size(); ++i) {
    result[4u * i] = static_cast<std::uint8_t>(state[i] >> 24u);
    result[4u * i + 1u] = static_cast<std::uint8_t>(state[i] >> 16u);
    result[4u * i + 2u] = static_cast<std::uint8_t>(state[i] >> 8u);
    result[4u * i + 3u] = static_cast<std::uint8_t>(state[i]);
  }
  return result;
}

Digest hash(const std::vector<std::uint8_t>& data) {
  return hash(data.data(), data.size());
}

Digest hash(const std::string& data) {
  return hash(reinterpret_cast<const std::uint8_t*>(data.data()), data.size());
}

Digest128 prefix128(const Digest& digest) {
  Digest128 result{};
  std::copy_n(digest.begin(), result.size(), result.begin());
  return result;
}

template <typename DigestType>
std::string digestHex(const DigestType& digest) {
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const std::uint8_t byte : digest) {
    stream << std::setw(2) << static_cast<unsigned>(byte);
  }
  return stream.str();
}

std::string hex(const Digest& digest) { return digestHex(digest); }
std::string hex(const Digest128& digest) { return digestHex(digest); }

}  // namespace VIO::dcreg_shadow::sha256
