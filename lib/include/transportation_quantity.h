#pragma once

#include <ostream>

namespace vrp {
/// Transportation Quantity representation
class TransportationQuantity {
    inline bool incomparable(TransportationQuantity other) const noexcept {
        return (volume < other.volume && weight > other.weight) ||
               (volume > other.volume && weight < other.weight) ||
               (volume == other.volume && weight != other.weight) ||
               (volume != other.volume && weight == other.weight);
    }

public:
    int volume = 0;
    int weight = 0;

    inline bool operator<(TransportationQuantity other) const noexcept {
        return volume < other.volume && weight < other.weight;
    }

    inline bool operator>(TransportationQuantity other) const noexcept {
        return volume > other.volume && weight > other.weight;
    }

    inline bool operator<=(TransportationQuantity other) const noexcept {
        return volume <= other.volume && weight <= other.weight;
    }

    inline bool operator>=(TransportationQuantity other) const noexcept {
        return volume >= other.volume && weight >= other.weight;
    }

    // Note: The last ones are complicated. In some sense, == is equal to !=
    // Why: if we can't really compare the objects (say whether one < another),
    // those objects are incomparable and thus "equivalent" to us
    inline bool operator!=(TransportationQuantity other) const noexcept {
        return (volume != other.volume && weight != other.weight) ||
               incomparable(other);
    }

    inline bool operator==(TransportationQuantity other) const noexcept {
        return (volume == other.volume && weight == other.weight) ||
               incomparable(other);
    }

    inline TransportationQuantity&
    operator-=(TransportationQuantity other) noexcept {
        this->volume -= other.volume;
        this->weight -= other.weight;
        return *this;
    }

    inline TransportationQuantity
    operator-(TransportationQuantity other) noexcept {
        return {this->volume - other.volume, this->weight - other.weight};
    }

    inline TransportationQuantity&
    operator+=(TransportationQuantity other) noexcept {
        this->volume += other.volume;
        this->weight += other.weight;
        return *this;
    }

    inline TransportationQuantity
    operator+(TransportationQuantity other) noexcept {
        return {this->volume + other.volume, this->weight + other.weight};
    }

    inline TransportationQuantity operator+(int value) const noexcept {
        return {this->volume + value, this->weight + value};
    }

    inline operator bool() const noexcept {
        // true if both volume and weight != 0, false otherwise
        return this->volume != 0 && this->weight != 0;
    }

    inline TransportationQuantity operator+(TransportationQuantity other) const
        noexcept {
        return TransportationQuantity{this->volume + other.volume,
                                      this->weight + other.weight};
    }

    inline TransportationQuantity operator/(TransportationQuantity other) const
        noexcept {
        return TransportationQuantity{this->volume / other.volume,
                                      this->weight / other.weight};
    }

    inline TransportationQuantity operator*(int v) const noexcept {
        return TransportationQuantity{this->volume * v, this->weight * v};
    }

    inline TransportationQuantity operator*(double v) const noexcept {
        return TransportationQuantity{static_cast<int>(this->volume * v),
                                      static_cast<int>(this->weight * v)};
    }

    inline bool operator==(int v) const noexcept {
        return this->volume == v && this->weight == v;
    }
};

inline TransportationQuantity operator*(int v,
                                        TransportationQuantity q) noexcept {
    return q * v;
}
}  // namespace vrp

namespace std {
inline ostream& operator<<(ostream& o, vrp::TransportationQuantity q) {
    return o << "{" << q.volume << ", " << q.weight << "}";
}
}  // namespace std
