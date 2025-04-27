const summon = async (lng: number, lat: number) => {
  return fetch("http://localhost:8000/api/summon", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
      Accept: "application/json",
    },
    body: JSON.stringify({
      lon: lng,
      lat,
    }),
  });
};

export { summon };
