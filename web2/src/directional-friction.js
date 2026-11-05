(() => {
  const nonnegative = value => Math.max(0, Number(value) || 0);
  const directionKey = (localVelocity, fallbackDirection = 1) =>
    (localVelocity === 0 ? fallbackDirection : localVelocity) >= 0 ? 'positive' : 'negative';
  const directionalValue = (branches, localVelocity, fallbackDirection = 1) =>
    nonnegative(branches?.[directionKey(localVelocity, fallbackDirection)]);
  const frictionMagnitude = (branches, localVelocity, fallbackDirection = 1) => {
    const branch = branches?.[directionKey(localVelocity, fallbackDirection)] || {};
    return nonnegative(branch.coulomb) + nonnegative(branch.viscous) * Math.abs(localVelocity);
  };
  window.DirectionalFriction = { nonnegative, directionKey, directionalValue, frictionMagnitude };
})();
