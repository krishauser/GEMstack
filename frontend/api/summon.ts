const summon = async (lng: number, lat: number) => {
  return fetch(`${process.env.NEXT_PUBLIC_API_BASE_URL}/api/summon`, {
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
