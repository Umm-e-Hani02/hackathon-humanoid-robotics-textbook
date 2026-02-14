# Production Deployment Guide

## Overview
This guide covers deploying the Physical AI Book project to production with:
- **Backend**: Render.com (FastAPI)
- **Frontend**: Vercel (Docusaurus)
- **Database**: Qdrant Cloud
- **AI**: Cohere API

---

## Step 1: Deploy Backend to Render.com

### 1.1 Create New Web Service
1. Go to https://dashboard.render.com
2. Click **New** → **Web Service**
3. Connect your GitHub repository: `Umm-e-Hani02/hackathon-humanoid-robotics-textbook`
4. Configure the service:
   - **Name**: `hackathon-humanoid-robotics-textbook`
   - **Region**: Choose closest to your Qdrant region (EU West recommended)
   - **Branch**: `main`
   - **Root Directory**: `backend`
   - **Runtime**: `Python 3`
   - **Build Command**: `pip install -r requirements.txt`
   - **Start Command**: `uvicorn main:app --host 0.0.0.0 --port 8001`
   - **Instance Type**: `Free`

### 1.2 Configure Environment Variables
In the Render dashboard, go to **Environment** tab and add these variables:

```
QDRANT_URL
https://cbc2aaae-27cc-403b-a0ed-3144cc479c71.eu-west-2-0.aws.cloud.qdrant.io

QDRANT_API_KEY
eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.OxKxFQODXc65Vx1MCuL5x1JGOrZpdFATe5vM0oRPGlA

QDRANT_COLLECTION_NAME
physical-ai-book-rag

COHERE_API_KEY
frc9a5gzyHUpLFfbkQQK6qDsKLaD7nqVwBOJ1xr0

FRONTEND_URL
https://your-app.vercel.app
```

**Important**:
- Do NOT include quotes around values
- Add `FRONTEND_URL` after you deploy to Vercel (Step 2)
- All values should be on a single line

### 1.3 Deploy
1. Click **Create Web Service**
2. Wait for deployment to complete (5-10 minutes)
3. Your backend URL will be: `https://hackathon-humanoid-robotics-textbook.onrender.com`

### 1.4 Verify Backend
Test the backend is working:

```bash
curl https://hackathon-humanoid-robotics-textbook.onrender.com/
```

Expected response:
```json
{"message":"RAG pipeline backend is running."}
```

Test the chat endpoint:
```bash
curl -X POST https://hackathon-humanoid-robotics-textbook.onrender.com/agent/chat \
  -H "Content-Type: application/json" \
  -d '{"text":"What is Physical AI?"}'
```

---

## Step 2: Deploy Frontend to Vercel

### 2.1 Prepare Frontend Configuration
The frontend is already configured to use environment variables. No code changes needed.

### 2.2 Deploy to Vercel

#### Option A: Using Vercel CLI (Recommended)
1. Install Vercel CLI:
   ```bash
   npm install -g vercel
   ```

2. Login to Vercel:
   ```bash
   vercel login
   ```

3. Deploy from the frontend directory:
   ```bash
   cd physical-ai-and-humanoid-robots
   vercel
   ```

4. Follow the prompts:
   - **Set up and deploy**: Yes
   - **Which scope**: Your account
   - **Link to existing project**: No
   - **Project name**: `physical-ai-book` (or your choice)
   - **Directory**: `./` (current directory)
   - **Override settings**: No

5. Deploy to production:
   ```bash
   vercel --prod
   ```

#### Option B: Using Vercel Dashboard
1. Go to https://vercel.com/new
2. Import your GitHub repository: `Umm-e-Hani02/hackathon-humanoid-robotics-textbook`
3. Configure project:
   - **Framework Preset**: Docusaurus
   - **Root Directory**: `physical-ai-and-humanoid-robots`
   - **Build Command**: `npm run build`
   - **Output Directory**: `build`
   - **Install Command**: `npm install`

4. Add Environment Variable (optional override):
   - **Name**: `API_URL`
   - **Value**: `https://hackathon-humanoid-robotics-textbook.onrender.com/agent/chat`
   - Note: This is optional since the code already has this as default for production

5. Click **Deploy**

### 2.3 Get Your Vercel URL
After deployment completes, you'll get a URL like:
- `https://physical-ai-book.vercel.app`
- Or `https://physical-ai-book-username.vercel.app`

---

## Step 3: Update Backend CORS

Now that you have your Vercel URL, update the backend:

1. Go to Render dashboard: https://dashboard.render.com
2. Select your service: `hackathon-humanoid-robotics-textbook`
3. Go to **Environment** tab
4. Update `FRONTEND_URL` variable:
   ```
   FRONTEND_URL
   https://your-actual-vercel-url.vercel.app
   ```
5. Click **Save Changes**
6. Render will automatically redeploy

---

## Step 4: Test Production Setup

### 4.1 Test Backend
```bash
curl https://hackathon-humanoid-robotics-textbook.onrender.com/agent/chat \
  -H "Content-Type: application/json" \
  -d '{"text":"What is Physical AI?"}'
```

### 4.2 Test Frontend
1. Open your Vercel URL: `https://your-app.vercel.app`
2. Navigate to any chapter
3. Select text with your mouse
4. Open chatbot (🤖 icon)
5. Verify "📄 Selected text will be used as context" appears
6. Ask a question
7. Verify you get a proper response

### 4.3 Test Text Selection Feature
1. Go to "What is Physical AI?" chapter
2. Select this text: "Physical AI refers to artificial intelligence systems..."
3. Open chatbot
4. Ask: "Explain this concept"
5. Verify the response uses your selected text as context

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                     Production Architecture                  │
└─────────────────────────────────────────────────────────────┘

┌──────────────┐         ┌──────────────┐         ┌──────────────┐
│   Vercel     │         │  Render.com  │         │ Qdrant Cloud │
│  (Frontend)  │────────▶│  (Backend)   │────────▶│  (Vectors)   │
│              │  HTTPS  │   FastAPI    │  HTTPS  │   141 docs   │
│  Docusaurus  │         │              │         │              │
└──────────────┘         └──────────────┘         └──────────────┘
                                │
                                │ HTTPS
                                ▼
                         ┌──────────────┐
                         │  Cohere API  │
                         │  (AI Model)  │
                         │  Embeddings  │
                         └──────────────┘

Environment Variables:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Frontend (Vercel):
  - NODE_ENV=production (automatic)
  - API_URL (optional, has default)

Backend (Render):
  - QDRANT_URL
  - QDRANT_API_KEY
  - QDRANT_COLLECTION_NAME
  - COHERE_API_KEY
  - FRONTEND_URL
```

---

## Environment Configuration Summary

### Development (Local)
```javascript
// Frontend automatically uses:
apiUrl: 'http://127.0.0.1:8001/agent/chat'

// Backend reads from .env file:
QDRANT_URL=https://...
QDRANT_API_KEY=...
COHERE_API_KEY=...
```

### Production
```javascript
// Frontend automatically uses:
apiUrl: 'https://hackathon-humanoid-robotics-textbook.onrender.com/agent/chat'

// Backend reads from Render environment variables:
QDRANT_URL=https://...
QDRANT_API_KEY=...
COHERE_API_KEY=...
FRONTEND_URL=https://your-app.vercel.app
```

---

## Troubleshooting

### Backend Issues

#### Error: "An unexpected error occurred"
**Check Render logs:**
1. Go to Render dashboard
2. Click your service
3. Go to **Logs** tab
4. Look for error messages

**Common causes:**
- Missing environment variables
- Qdrant connection timeout
- Cohere API rate limit

#### Error: CORS blocked
**Solution:**
1. Verify `FRONTEND_URL` is set correctly in Render
2. Make sure it matches your Vercel URL exactly
3. Redeploy backend after updating

### Frontend Issues

#### Error: "Failed to fetch"
**Check:**
1. Backend is running (visit backend URL in browser)
2. CORS is configured correctly
3. Network tab in browser DevTools for actual error

#### Text selection not working
**Check:**
1. Browser console for JavaScript errors
2. Verify selected text appears in chatbot UI
3. Check Network tab to see if `selected_text` is being sent

---

## Security Best Practices

✅ **Implemented:**
- Environment variables for all secrets
- CORS restricted to specific origins
- HTTPS for all connections
- No hardcoded credentials

✅ **Recommended:**
- Rotate API keys periodically
- Monitor Cohere API usage
- Set up Render alerts for errors
- Enable Vercel deployment protection

---

## Monitoring & Maintenance

### Check Backend Health
```bash
curl https://hackathon-humanoid-robotics-textbook.onrender.com/
```

### Monitor Logs
- **Render**: Dashboard → Service → Logs
- **Vercel**: Dashboard → Project → Deployments → View Function Logs

### Update Dependencies
```bash
# Backend
cd backend
pip list --outdated

# Frontend
cd physical-ai-and-humanoid-robots
npm outdated
```

---

## Cost Breakdown

| Service | Plan | Cost |
|---------|------|------|
| Render.com | Free | $0/month |
| Vercel | Hobby | $0/month |
| Qdrant Cloud | Free tier | $0/month |
| Cohere API | Trial/Free | $0/month* |

*Monitor Cohere usage to avoid unexpected charges

---

## Next Steps After Deployment

1. ✅ Test all features thoroughly
2. ✅ Share your project URL
3. ✅ Monitor logs for first 24 hours
4. ✅ Set up custom domain (optional)
5. ✅ Add analytics (optional)

---

## Support

If you encounter issues:
1. Check Render logs for backend errors
2. Check browser console for frontend errors
3. Verify all environment variables are set correctly
4. Test backend endpoint directly with curl
5. Ensure Qdrant cluster is active

---

**Deployment Complete! 🎉**

Your Physical AI Book is now live with:
- ✅ Production-ready backend
- ✅ Fast, scalable frontend
- ✅ Text selection feature
- ✅ RAG-powered chatbot
- ✅ Secure configuration
