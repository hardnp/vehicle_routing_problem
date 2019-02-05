#pragma once

namespace vrp {
/// Transportation Quantity representation
class TransportationQuantity {
    inline bool incomparable(TransportationQuantity other) const {
        return (volume < other.volume && weight > other.weight)
            || (volume > other.volume && weight < other.weight)
            || (volume == other.volume && weight != other.weight)
            || (volume != other.volume && weight == other.weight);
    }
public:
    int volume = 0;
    int weight = 0;

    inline bool operator<(TransportationQuantity other) const {
        return volume < other.volume && weight < other.weight;
    }

    inline bool operator>(TransportationQuantity other) const {
        return volume > other.volume && weight > other.weight;
    }

    inline bool operator<=(TransportationQuantity other) const {
        return volume <= other.volume && weight <= other.weight;
    }

    inline bool operator>=(TransportationQuantity other) const {
        return volume >= other.volume && weight >= other.weight;
    }

    // Note: The last ones are complicated. In some sense, == is equal to !=
    // Why: if we can't really compare the objects (say whether one < another),
    // those objects are incomparable and thus "equivalent" to us
    inline bool operator!=(TransportationQuantity other) const {
        return (volume != other.volume && weight != other.weight)
            || incomparable(other);
    }

    inline bool operator==(TransportationQuantity other) const {
        return (volume == other.volume && weight == other.weight)
            || incomparable(other);
    }
};
}  // vrp
