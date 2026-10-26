"""Learned RGB vehicle type/color heads, independent of task and actor metadata."""
from pathlib import Path
import cv2,numpy as np,torch
from torch import nn
from torchvision.models import mobilenet_v3_large,MobileNet_V3_Large_Weights,resnet18,ResNet18_Weights

TYPE_NAMES=('van','car','truck','bus','non_vehicle')
COLOR_NAMES=('red','blue','white','black','gray','yellow')


def letterbox_rgb(image_bgr,size=192):
    height,width=image_bgr.shape[:2]
    if not height or not width:raise ValueError('Empty attribute crop')
    scale=min(size/width,size/height)
    image=cv2.resize(image_bgr,(max(1,round(width*scale)),max(1,round(height*scale))),interpolation=cv2.INTER_LINEAR)
    canvas=np.full((size,size,3),127,np.uint8)
    y=(size-image.shape[0])//2;x=(size-image.shape[1])//2;canvas[y:y+image.shape[0],x:x+image.shape[1]]=image
    return cv2.cvtColor(canvas,cv2.COLOR_BGR2RGB)


class VehicleAttributeNet(nn.Module):
    def __init__(self,pretrained=False,architecture='mobilenet_v3_large'):
        super().__init__()
        if architecture=='mobilenet_v3_large':
            self.backbone=mobilenet_v3_large(weights=MobileNet_V3_Large_Weights.IMAGENET1K_V2 if pretrained else None)
            self.backbone.classifier[-1]=nn.Identity();features=1280
        elif architecture=='resnet18':
            self.backbone=resnet18(weights=ResNet18_Weights.IMAGENET1K_V1 if pretrained else None)
            self.backbone.fc=nn.Identity();features=512
        else:raise ValueError(f'Unsupported attribute backbone: {architecture}')
        self.type_head=nn.Linear(features,len(TYPE_NAMES));self.color_head=nn.Linear(features,len(COLOR_NAMES))

    def forward(self,images):
        features=self.backbone(images)
        return self.type_head(features),self.color_head(features)


class VehicleAttributeRecognizer:
    """One reusable image-only network; confidences/unknown rules are calibrated outside."""
    _cache={}

    def __init__(self,path,device=0):
        self.path=Path(path).resolve()
        self.device=torch.device(f'cuda:{device}' if str(device).isdigit() else str(device))
        key=(str(self.path),str(self.device))
        if key not in self._cache:
            checkpoint=torch.load(self.path,map_location='cpu',weights_only=False)
            if tuple(checkpoint['type_names'])!=TYPE_NAMES or tuple(checkpoint['color_names'])!=COLOR_NAMES:
                raise ValueError('Attribute label ontology mismatch')
            model=VehicleAttributeNet(architecture=checkpoint.get('architecture','mobilenet_v3_large'));model.load_state_dict(checkpoint['state_dict']);model.to(self.device).eval()
            self._cache[key]=(model,checkpoint)
        self.model,self.checkpoint=self._cache[key]
        self.size=self.checkpoint.get('image_size',192)
        self.mean=torch.tensor([.485,.456,.406],device=self.device).view(1,3,1,1)
        self.std=torch.tensor([.229,.224,.225],device=self.device).view(1,3,1,1)

    def predict(self,image_bgr,boxes):
        if not boxes:return []
        h,w=image_bgr.shape[:2];images=[]
        for box in boxes:
            x1,y1,x2,y2=box;pad=max(2,round(min(x2-x1,y2-y1)*.08))
            left=max(0,int(x1)-pad);top=max(0,int(y1)-pad);right=min(w,int(np.ceil(x2))+pad);bottom=min(h,int(np.ceil(y2))+pad)
            crop=image_bgr[top:bottom,left:right]
            if not crop.size:crop=np.full((8,8,3),127,np.uint8)
            images.append(letterbox_rgb(crop,self.size))
        batch=torch.from_numpy(np.stack(images)).permute(0,3,1,2).to(self.device,dtype=torch.float32)/255.
        batch=(batch-self.mean)/self.std
        with torch.inference_mode():
            types,colors=self.model(batch)
            types=(types/self.checkpoint.get('type_temperature',1.)).softmax(1).cpu().numpy()
            colors=(colors/self.checkpoint.get('color_temperature',1.)).softmax(1).cpu().numpy()
        return [{'type_probabilities':{k:float(v) for k,v in zip(TYPE_NAMES,t)},
                 'color_probabilities':{k:float(v) for k,v in zip(COLOR_NAMES,c)},
                 'predicted_type':TYPE_NAMES[int(t.argmax())],'type_probability':float(t.max()),
                 'predicted_color':COLOR_NAMES[int(c.argmax())],'color_probability':float(c.max()),
                 'attribute_source':self.checkpoint.get('architecture','mobilenet_v3_large')+'_dual_attribute_heads'} for t,c in zip(types,colors)]
