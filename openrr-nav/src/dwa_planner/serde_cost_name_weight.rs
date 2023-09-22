use std::collections::HashMap;

use serde::{Deserialize, Deserializer, Serialize, Serializer};

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct CostNameWeightPair {
    name: String,
    value: f64,
}

pub fn serialize<S: Serializer>(
    data: &HashMap<String, f64>,
    serializer: S,
) -> Result<S::Ok, S::Error> {
    let pairs = data
        .iter()
        .map(|(name, value)| CostNameWeightPair {
            name: name.clone(),
            value: *value,
        })
        .collect::<Vec<_>>();
    pairs.serialize(serializer)
}

pub fn deserialize<'de, D: Deserializer<'de>>(
    deserializer: D,
) -> Result<HashMap<String, f64>, D::Error> {
    let pairs = Vec::<CostNameWeightPair>::deserialize(deserializer)?;
    let map = pairs.into_iter().map(|c| (c.name, c.value)).collect();
    Ok(map)
}
