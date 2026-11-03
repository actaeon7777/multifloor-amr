"""DINO dependencies are loaded only by the inference interpreter."""
import numpy as np

# 2026-11-02 offline comparison on the recorded arrival frames: DINOv3 ViT-S widened the
# 3F/4F score gap 2-5x over DINOv2-small (43/45 vs 41/45 frames) and ran faster (153 vs
# 181 ms/frame on 2 CPU threads). DINOv3 needs the gated download (accepted) once.
DEFAULT_MODEL = 'facebook/dinov3-vits16-pretrain-lvd1689m'


def centre_square(image, side=224):
    """Shortest side to ``side``, centre crop: the hall seen through the door, without
    squashing 16:9 frames (DINOv3's processor would) and cropping the door panels away."""
    width, height = image.size
    scale = side / min(width, height)
    image = image.resize((max(side, round(width * scale)), max(side, round(height * scale))))
    width, height = image.size
    left, top = (width - side) // 2, (height - side) // 2
    return image.crop((left, top, left + side, top + side))


class DinoEmbedder:
    def __init__(self, model_id=DEFAULT_MODEL, threads=2):
        import torch
        from transformers import AutoImageProcessor, AutoModel
        torch.set_num_threads(threads)
        self.torch = torch
        self.processor = AutoImageProcessor.from_pretrained(model_id, local_files_only=True)
        self.model = AutoModel.from_pretrained(model_id, local_files_only=True).eval().to('cpu')
        # DINOv3 puts register tokens after CLS; they are not image patches.
        self.skip = 1 + int(getattr(self.model.config, 'num_register_tokens', 0) or 0)

    def embed(self, images):
        with self.torch.inference_mode():
            inputs = self.processor(images=[centre_square(image.convert('RGB')) for image in images],
                                    return_tensors='pt')
            tokens = self.model(**inputs).last_hidden_state
            features = self.torch.cat((tokens[:, 0], tokens[:, self.skip:].mean(dim=1)), dim=1)
            features = self.torch.nn.functional.normalize(features, dim=1)
            return features.cpu().numpy().astype(np.float32)
