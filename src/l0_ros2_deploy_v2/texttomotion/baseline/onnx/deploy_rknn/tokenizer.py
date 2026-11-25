"""Portable NumPy character preprocessing; no PyTorch or custom .so required."""
import numpy as np


def fixed_tokenize(text: str, char_vocab: dict, max_length: int):
    if not isinstance(text, str) or not text.strip():
        raise ValueError('Expected a nonempty text string')
    ids = np.full((1, max_length + 1), char_vocab['<pad>'], np.int32)
    padding = np.ones_like(ids, dtype=np.int32)
    encoded = [char_vocab['<cls>'], *[
        char_vocab.get(character, char_vocab['<unk>']) for character in text[:max_length]
    ]]
    ids[0, :len(encoded)] = encoded
    padding[0, :len(encoded)] = 0
    return {'input_ids': ids, 'padding_mask': padding}
