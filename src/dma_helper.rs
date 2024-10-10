#![allow(unused)]

/// Trait which is implemented by anything that can be read via DMA.
///
/// # Safety
///
/// The implementing type must be safe to use for DMA reads. This means:
///
/// - The range returned by rx_address_count must point to a valid address,
///   and if rx_increment is true, count must fit into the allocated buffer.
/// - As long as no `&mut self` method is called on the implementing object:
///   - `rx_address_count` must always return the same value, if called multiple
///     times.
///   - The memory specified by the pointer and size returned by `rx_address_count`
///     must not be freed during the transfer it is used in as long as `self` is not dropped.
pub unsafe trait DmaReadTarget {
    /// Type which is transferred in a single DMA transfer.
    type ReceivedWord;

    /// Returns the DREQ number for this data source (`None` for memory buffers).
    fn rx_treq(&self) -> Option<u8>;

    /// Returns the address and the maximum number of words that can be transferred from this data
    /// source in a single DMA operation.
    ///
    /// For peripherals, the count should likely be u32::MAX. If a data source implements
    /// EndlessReadTarget, it is suitable for infinite transfers from or to ring buffers. Note that
    /// ring buffers designated for endless transfers, but with a finite buffer size, should return
    /// the size of their individual buffers here.
    ///
    /// # Safety
    ///
    /// This function has the same safety guarantees as `ReadBuffer::read_buffer`.
    fn rx_address_count(&self) -> (u32, u32);

    /// Returns whether the address shall be incremented after each transfer.
    fn rx_increment(&self) -> bool;
}

/// Trait which is implemented by anything that can be written via DMA.
///
/// # Safety
///
/// The implementing type must be safe to use for DMA writes. This means:
///
/// - The range returned by tx_address_count must point to a valid address,
///   and if tx_increment is true, count must fit into the allocated buffer.
/// - As long as no other `&mut self` method is called on the implementing object:
///   - `tx_address_count` must always return the same value, if called multiple
///     times.
///   - The memory specified by the pointer and size returned by `tx_address_count`
///     must not be freed during the transfer it is used in as long as `self` is not dropped.
pub unsafe trait DmaWriteTarget {
    /// Type which is transferred in a single DMA transfer.
    type TransmittedWord;

    /// Returns the DREQ number for this data sink (`None` for memory buffers).
    fn tx_treq(&self) -> Option<u8>;

    /// Returns the address and the maximum number of words that can be transferred from this data
    /// source in a single DMA operation.
    ///
    /// See `ReadTarget::rx_address_count` for a complete description of the semantics of this
    /// function.
    fn tx_address_count(&self) -> (u32, u32);

    /// Returns whether the address shall be incremented after each transfer.
    fn tx_increment(&self) -> bool;
}
