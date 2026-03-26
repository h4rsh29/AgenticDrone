from torch.cuda import temperature
import os
from dotenv import load_dotenv
from langchain_nvidia_ai_endpoints import ChatNVIDIA
from transformers import AutoProcessor, AutoModelForImageTextToText, BitsAndBytesConfig
import torch
from ultralytics import YOLO

load_dotenv()

# --- DETECTION: YOLO26 Nano ---
# Optimized for real-time surveillance with deterministic latency
print("Loading YOLO26n...")
yolo_model = YOLO('yolo26n.pt') 
print("YOLO26 loaded successfully!")

# --- COGNITION: GPT-OSS 120B (via OpenRouter) ---
# Optimized for 2026 agentic reasoning
logic_llm = ChatNVIDIA(
    model="openai/gpt-oss-120b",
    api_key=os.getenv("OPENAI_API_KEY"),
    temperature=0.1,

)

# --- PERCEPTION: SmolVLM (Hugging Face) ---
# Using SmolVLM-500M for optimal 4GB VRAM performance
model_id = "HuggingFaceTB/SmolVLM-500M-Instruct"
token = os.getenv("HF_TOKEN")
# 4-bit quantization to keep the model footprint under 1GB
quant_config = BitsAndBytesConfig(
    load_in_4bit=True,
    bnb_4bit_compute_dtype=torch.float16
)

print(f"Loading {model_id}...")
processor = AutoProcessor.from_pretrained(model_id, token=token)
vlm_model = AutoModelForImageTextToText.from_pretrained(
    model_id,
    quantization_config=quant_config,
    device_map="auto",
    token=token
)
print("Model loaded successfully on GPU!")
